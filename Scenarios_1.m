%% ---------------- SETUP ----------------
clearvars; clc;
%% Paths
addpath(genpath('C:\Users\namic\OneDrive - TU Eindhoven\Documents_1\Lectures\Charger\YALMIP-master\YALMIP-master'))
addpath('C:\gurobi1203\win64\matlab')

%% Define scenarios
scenarios = {
    % arrival_dist  amortyears  Peak_imp  Peak_exp  Ppv_percentage  veh_cap_bat
    1              6           1         1         1              200
    1              6           1         1         0.3            200
    0              6           1         1         1              300
    1              6           2         2         1              200
    1              2           1         1         1              200
};

nScenarios = size(scenarios,1);

results_all = cell(nScenarios,1);

%% Run each scenario
for i = 1:nScenarios
    fprintf('\n--- Running Scenario %d ---\n', i);

    arrival_dist   = scenarios{i,1};
    amortyears     = scenarios{i,2};
    Peak_imp       = scenarios{i,3};
    Peak_exp       = scenarios{i,4};
    Ppv_percentage  = scenarios{i,5};
    veh_cap_bat    = scenarios{i,6};

    % Run optimization
    results = optimize_microgrid(arrival_dist, amortyears, Peak_imp, Peak_exp, Ppv_percentage, veh_cap_bat);
    results_all{i} = results;

    %% Display summary
    disp("Optimal chargers per type:"); disp(results.N)
    disp("Battery size [kWh]:"); disp(results.Ebat)
    disp("PV size [kWp]:"); disp(results.Epv)
    disp("Grid connection [kW]:"); disp(results.Pgrid)

    %% PLOTS
    figure('Name',sprintf('Scenario %d: EV vs Grid',i));
    yyaxis left
    plot(sum(results.EVload,1),'k','LineWidth',1.5); hold on;
    ylabel('EV Load [kW]');
    yyaxis right
    plot(results.GridImport,'b','LineWidth',1.5);
    plot(results.GridExport,'r','LineWidth',1.5);
    ylabel('Grid Power [kW]');
    xlabel('Time step'); grid on;
    legend('EV Load','Grid Import','Grid Export');
    title(sprintf('Scenario %d: EV Load vs Grid',i));

    figure('Name',sprintf('Scenario %d: Battery',i));
    subplot(2,1,1);
    plot(results.SOC,'LineWidth',1.5);
    ylabel('SOC [kWh]'); xlabel('Time step'); grid on;
    title('Battery State of Charge');
    subplot(2,1,2);
    plot(results.Pbat_ch,'g','LineWidth',1.5); hold on;
    plot(results.Pbat_dis,'m','LineWidth',1.5);
    ylabel('Power [kW]'); xlabel('Time step'); grid on;
    legend('Charge','Discharge');
    title('Battery Charge/Discharge Power');

    % figure('Name',sprintf('Scenario %d: PV vs Curtailment',i));
    % plot(results.PVgen,'y','LineWidth',1.5); hold on;
    % plot(results.PVcurt,'r--','LineWidth',1.5);
    % xlabel('Time step'); ylabel('PV Power [kW]');
    % legend('PV Generation','Curtailment'); grid on;
    % title(sprintf('Scenario %d: PV Generation vs Curtailment',i));
end