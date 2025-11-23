% File: power_budget.m
P_solar = 1.23 * 0.634;  % Avg solar
P_load = 0.260;          % Avg consumption
P_net = P_solar - P_load;
t_charge = 1.85 / P_net; % Hours to charge 500mAh
fprintf('Net Power: %.3f W\n', P_net);
fprintf('Charge Time: %.1f hours\n', t_charge);