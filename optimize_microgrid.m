function results = optimize_microgrid(arrival_dist, amortyears, Peak_imp, Peak_exp, Ppv_percentage, veh_cap_bat)

%% Time horizon
dt_min = 10; dt = dt_min/60;
T = 24*60/dt_min;
t = (1:T)';

%% ---------------- VEHICLE ARRIVAL MODULE ----------------
switch arrival_dist
    case 0
        load("arrivalTimes_l10.mat");
        load("departureTimes_l10.mat");
        load("soc_3.mat")
    case 1
        load("arrivalTimes_l5.mat");
        load("departureTimes_l5.mat");
        load("soc_2.mat")
end

V = length(arrivalTimes); % number of vehicles

% Vehicle battery parameters
Cap_batt = veh_cap_bat*ones(V,1); % kWh capacity per truck
SoC_init = 0.2 + 0.3*SoC_init_const(V,1); % initial SoC
SoC_target = 0.8*ones(V,1);
Eneed = (SoC_target-SoC_init).*Cap_batt; % kWh demand
eta_ch = 0.95;

%% ---------------- ENERGY PRICE/PRODUCTION MODULE ----------------
run("price_timeseries.m") % loads c_imp, c_exp, c_cust
% Peak multipliers
c_imp = Peak_imp * c_imp;
c_exp = Peak_exp * c_exp;

%% ---------------- CHARGING STATION MODULE ----------------
K = 2; % charger types
chargerCapacity = [150; 300];
chargerCost = [40000; 80000];

x = binvar(V,K,T); % assign vehicle v to charger k at t
p_vt = sdpvar(V,T,'full'); % kW allocated
u = sdpvar(V,1); % unmet energy (not used if hard constraint)

%% ---------------- MICROGRID MODULE ----------------
Epv = sdpvar(1); Ebat = sdpvar(1); Pbat = sdpvar(1); Pgrid = sdpvar(1);
p_bat_ch = sdpvar(T,1); p_bat_dis = sdpvar(T,1); soc = sdpvar(T,1);
g_import = sdpvar(T,1); g_export = sdpvar(T,1); p_curt = sdpvar(T,1);

Ppv_perkWp = max(0,Ppv_percentage*sin(pi*(t-30)/T)); % normalized PV
Paux = zeros(T,1);

%% ---------------- CONSTRAINTS ----------------
Constraints = [];
Constraints = [Constraints, Epv >= 0, Ebat >= 0, Pbat >= 0, Pgrid >= 0];
Constraints = [Constraints, Epv + Pgrid >= 1e-3];
N = intvar(K,1);
Constraints = [Constraints, N >= 1]; 

% Vehicle service windows
for v=1:V
    for tt=1:T
        inWin = (tt>=arrivalTimes(v) && tt<=departureTimes(v));
        if ~inWin
            Constraints = [Constraints, sum(x(v,:,tt))==0, p_vt(v,tt)==0];
        else
            Constraints = [Constraints, p_vt(v,tt) <= sum(x(v,:,tt)).*max(chargerCapacity)];
        end
        Constraints = [Constraints, p_vt(v,tt) >= 0];
    end
    delivered = sum(p_vt(v,:))*dt*eta_ch;
    Constraints = [Constraints, delivered >= Eneed(v)];
end

% Charger occupancy
for tt=1:T
    for k=1:K
        Constraints = [Constraints, sum(x(:,k,tt)) <= N(k)];
    end
    Constraints = [Constraints, sum(p_vt(:,tt)) <= N'*chargerCapacity];
end

% Microgrid balance
C_rate = 0.5;                     % battery C-rate (0.5C = 2h full charge/discharge)
Pbat_max = C_rate * Ebat;         % max power based on capacity
for tt=1:T
    Ppv_t = Epv*Ppv_perkWp(tt);
    Constraints = [Constraints, g_import(tt)-g_export(tt)+...
        p_bat_dis(tt)-p_bat_ch(tt)+Ppv_t-p_curt(tt) == sum(p_vt(:,tt))+Paux(tt)];
    Constraints = [Constraints, g_import(tt)>=0, g_export(tt)>=0, ...
        p_bat_ch(tt)>=0, p_bat_dis(tt)>=0, p_curt(tt)>=0];
    Constraints = [Constraints, g_import(tt)<=Pgrid, g_export(tt)<=Pgrid];
    Constraints = [Constraints, p_bat_ch(tt)<=Pbat, p_bat_dis(tt)<=Pbat];
    Constraints = [Constraints, p_bat_ch(tt) <= Pbat_max];
    Constraints = [Constraints, p_bat_dis(tt) <= Pbat_max];
end
% Energy bounds (kWh)
Ebat_min = 10;        % set a small positive min to avoid div-by-zero, adjust as needed
Ebat_max = 5000;       % realistic max battery energy (kWh) - tune for your system
Constraints = [Constraints, Ebat_min <= Ebat <= Ebat_max];

% Power bounds (kW)
Pbat_min = 0;
Pbat_max_fixed = 3000; % absolute power cap (kW) - tune as needed
Constraints = [Constraints, Pbat_min <= Pbat <= Pbat_max_fixed];
% Battery dynamics
Constraints = [Constraints, 0<=soc<=Ebat];
% % % Constraints = [Constraints, soc(1)==0.5*Ebat + (0.95*p_bat_ch(1)-p_bat_dis(1)/0.95)*dt];
Constraints = [Constraints, soc(1) == 0];
for tt=1:T-1
    Constraints = [Constraints, soc(tt+1)==soc(tt)+(0.95*p_bat_ch(tt)-p_bat_dis(tt)/0.95)*dt];
end

%% ------------ CapEx Other Costs ------------
BatteryEnergyCost=200;
BatteryPowerCost=300;
PanelsEnergyCost=500;
GridPowerCost=300;

%% ---------------- OBJECTIVE ----------------
CapEx = (chargerCost'*N + BatteryEnergyCost*Ebat + BatteryPowerCost*Pbat ...
    + PanelsEnergyCost*Epv + GridPowerCost*Pgrid) / (365*amortyears);
OpEx = sum(c_imp.*(g_import'*dt)) - sum(c_exp.*(g_export'*dt));
Revenue = sum(c_cust.*sum(p_vt,1)*dt);

Objective = CapEx + OpEx - Revenue;

%% ---------------- SOLVE ----------------
ops = sdpsettings('solver','gurobi','verbose',1);
optimize(Constraints,Objective,ops);

%% ---------------- OUTPUTS ----------------
results.N = value(N);
results.Ebat = value(Ebat);
results.Epv = value(Epv);
results.Pgrid = value(Pgrid);

results.EVload = value(sum(p_vt,1))';         % total EV demand per timestep
results.GridImport = value(g_import);
results.GridExport = value(g_export);
results.SOC = 100*value(soc)/value(Ebat);
results.Pbat_ch = value(p_bat_ch);
results.Pbat_dis = value(p_bat_dis);
% disp("Optimal chargers per type:"); disp(results.N)
% disp("Battery size [kWh]:"); disp(results.Ebat)
% disp("PV size [kWp]:"); disp(results.Epv)
% disp("Grid connection [kW]:"); disp(results.Pgrid)

end
