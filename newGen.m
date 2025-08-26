%% Clear workspace
clearvars; clc;

%% Paths
addpath(genpath('C:\Users\20220978\Downloads\YALMIP'))
addpath('C:\gurobi1203\win64\matlab')
run("price_timeseries.m")

%% Time horizon
dt_min = 10; dt = dt_min/60;
T = 24*60/dt_min;
t = (1:T)';
 
%% ---------------- VEHICLE ARRIVAL MODULE ----------------
% lambda = 0.5; % average arrivals per hour (example)
% inter_arrival = exprnd(1/lambda,[100,1]); % exponential inter-arrival
% arrivalTimes = cumsum(inter_arrival*60/dt_min); % in slots
% arrivalTimes = arrivalTimes(arrivalTimes<=T);
lambda = 0.5;           % average arrivals per hour
Vmax   = 100;          
% Expected arrivals per slot (dt_min minutes)
lambda_slot = lambda * dt;  
% Generate arrivals per slot
arrivals_per_slot = poissrnd(lambda_slot, T, 1);
% Convert to individual vehicle arrival times
arrivalTimes = [];
for tt = 1:T
    if arrivals_per_slot(tt) > 0
        arrivalTimes = [arrivalTimes; tt*ones(arrivals_per_slot(tt),1)];
    end
end
 
if length(arrivalTimes) > Vmax
    arrivalTimes = arrivalTimes(1:Vmax);
end
 
V = length(arrivalTimes); % number of vehicles
departureTimes = arrivalTimes + randi([6,30],V,1);
departureTimes = min(departureTimes,T);
 
% Vehicle battery parameters
Cap_batt = 200*ones(V,1); % kWh capacity per truck
SoC_init = 0.2 + 0.3*rand(V,1); % initial SoC
SoC_target = 0.8*ones(V,1);
Eneed = (SoC_target-SoC_init).*Cap_batt; % kWh demand
eta_ch = 0.95;
 
%% ---------------- ENERGY PRICE/PRODUCTION MODULE ----------------
% c_imp = 0.15 + 0.05*sin(2*pi*t/T); % €/kWh import
% c_exp = 0.05*ones(T,1);            % €/kWh export
% m = 0.10;
% c_cust = c_imp + m;            % customer price
 
%% ---------------- CHARGING STATION MODULE ----------------
K = 2; % charger types
chargerCapacity = [150; 300];
chargerCost = [40000; 80000];
 
x = binvar(V,K,T); % assign vehicle v to charger k at t
p_vt = sdpvar(V,T,'full'); % kW allocated
u = sdpvar(V,1); % unmet energy
 
%% ---------------- MICROGRID MODULE ----------------
Epv = sdpvar(1); Ebat = sdpvar(1); Pbat = sdpvar(1); Pgrid = sdpvar(1);
p_bat_ch = sdpvar(T,1); p_bat_dis = sdpvar(T,1); soc = sdpvar(T,1);
g_import = sdpvar(T,1); g_export = sdpvar(T,1); p_curt = sdpvar(T,1);
 
Ppv_perkWp = max(0,sin(pi*(t-30)/T)); % normalized PV
Paux = zeros(T,1);
 
%% ---------------- CONSTRAINTS ----------------
Constraints = [];
% Investment variable bounds
Constraints = [Constraints, Epv >= 0, Ebat >= 0, Pbat >= 0, Pgrid >= 0];
N = intvar(K,1);
Constraints = [Constraints, N >= 0]; % nonnegative number of chargers
% Vehicle service windows
for v=1:V
    delivered = 0;
    for tt=1:T
        inWin = (tt>=arrivalTimes(v) && tt<=departureTimes(v));
        if ~inWin
            Constraints = [Constraints, sum(x(v,:,tt))==0, p_vt(v,tt)==0];
        else
            Constraints = [Constraints, p_vt(v,tt) <= sum(x(v,:,tt)).*max(chargerCapacity)];
        end
        Constraints = [Constraints, p_vt(v,tt) >= 0];
    end
    % Energy balance
    delivered = sum(p_vt(v,:))*dt*eta_ch;
    %%%Constraints = [Constraints, delivered + u(v) >= Eneed(v), u(v)>=0];
    % HARD energy requirement (no slack):
    Constraints = [Constraints, delivered >= Eneed(v)];
end
 
% Charger occupancy
%%N = intvar(K,1);
for tt=1:T
    for k=1:K
        Constraints = [Constraints, sum(x(:,k,tt)) <= N(k)];
    end
    Constraints = [Constraints, sum(p_vt(:,tt)) <= N'*chargerCapacity];
end
 
% Microgrid balance
for tt=1:T
    Ppv_t = Epv*Ppv_perkWp(tt);
    Constraints = [Constraints, g_import(tt)-g_export(tt)+...
        p_bat_dis(tt)-p_bat_ch(tt)+Ppv_t-p_curt(tt) == sum(p_vt(:,tt))+Paux(tt)];
    Constraints = [Constraints, g_import(tt)>=0, g_export(tt)>=0, ...
        p_bat_ch(tt)>=0, p_bat_dis(tt)>=0, p_curt(tt)>=0];
    Constraints = [Constraints, g_import(tt)<=Pgrid, g_export(tt)<=Pgrid];
    Constraints = [Constraints, p_bat_ch(tt)<=Pbat, p_bat_dis(tt)<=Pbat];
end
 
% Battery dynamics
Constraints = [Constraints, 0<=soc<=Ebat];
Constraints = [Constraints, soc(1)==0.5*Ebat + (0.95*p_bat_ch(1)-p_bat_dis(1)/0.95)*dt];
for tt=1:T-1
    Constraints = [Constraints, soc(tt+1)==soc(tt)+(0.95*p_bat_ch(tt)-p_bat_dis(tt)/0.95)*dt];
end
%% ------------ CapEx Other Costs ------------
amortyears=2;
BatteryEnergyCost=100;
BatteryPowerCost=200;
PanelsEnergyCost=700
GridPowerCost=150
%% ---------------- OBJECTIVE ----------------
CapEx = (chargerCost'*N + BatteryEnergyCost*Ebat + BatteryPowerCost*Pbat ...
    + PanelsEnergyCost*Epv + GridPowerCost*Pgrid) / (365*amortyears);
OpEx = sum(c_imp.*(g_import*dt)) - sum(c_exp.*(g_export*dt));
Revenue = sum(c_cust.*sum(p_vt,1)'*dt);
Penalty = 100*sum(u);
 
%%%Objective = CapEx + OpEx - Revenue + Penalty;
Objective = CapEx + OpEx - Revenue;
%%Objective = Revenue - (CapEx + OpEx);
 
%% ---------------- SOLVE ----------------
ops = sdpsettings('solver','gurobi','verbose',1);
optimize(Constraints,Objective,ops);
 
%% ---------------- OUTPUTS ----------------
disp("Optimal chargers per type:"); disp(value(N))
disp("Battery size [kWh]:"); disp(value(Ebat))
disp("PV size [kWp]:"); disp(value(Epv))
disp("Grid connection [kW]:"); disp(value(Pgrid))
%%%disp("Unserved demand [kWh]:"); disp(sum(value(u)))