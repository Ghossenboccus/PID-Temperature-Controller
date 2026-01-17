%% PID Controller Tuning
% PID Temperature Controller Project
% Author: Gabriel Hossenboccus
%
% This script calculates PID controller gains using various tuning methods:
%   1. Ziegler-Nichols (Z-N) Open Loop Method
%   2. Cohen-Coon Method
%   3. AMIGO Method (for PI control)
%   4. Manual tuning recommendations
%
% Prerequisites:
%   - Run system_model.m first to identify K and tau
%   - Or manually enter K and tau values below
%
% Output:
%   - Kp, Ki, Kd gains for different methods
%   - Closed-loop simulation and comparison

%% Clear workspace
clear all;
close all;
clc;

%% System Parameters (from system_model.m)
fprintf('========================================\n');
fprintf('PID Controller Tuning\n');
fprintf('========================================\n\n');

% Option 1: Load from saved file
params_file = '../results/system_parameters.txt';
if isfile(params_file)
    fprintf('Loading system parameters from file...\n');
    % Read K and tau from file (you'll need to parse it)
    % For now, prompt user to enter manually
end

% Option 2: Enter manually
fprintf('Enter system parameters (from system identification):\n');
K_input = input('  Steady-state gain K (°C): ');
tau_input = input('  Time constant τ (s): ');

% Use input values or defaults for testing
if isempty(K_input) || isempty(tau_input)
    fprintf('\nUsing default values for demonstration:\n');
    K = 30;      % °C
    tau = 90;    % seconds
else
    K = K_input;
    tau = tau_input;
end

fprintf('\n=== System Parameters ===\n');
fprintf('K = %.2f °C\n', K);
fprintf('τ = %.2f s\n\n', tau);

% Create transfer function
G = tf(K, [tau, 1]);

%% Method 1: Ziegler-Nichols Open Loop Method

fprintf('=== Ziegler-Nichols Open Loop ===\n');
fprintf('For first-order system: G(s) = K/(τs + 1)\n\n');

% Z-N formulas for first-order without dead time
% Note: Classical Z-N assumes dead time L. For pure first-order, approximate L ≈ 0.1*tau

L = 0.1 * tau;  % Approximate dead time (small for thermal systems)

% P Controller
Kp_ZN_P = tau / (K * L);

% PI Controller
Kp_ZN_PI = 0.9 * tau / (K * L);
Ki_ZN_PI = Kp_ZN_PI / (3.3 * L);

% PID Controller
Kp_ZN_PID = 1.2 * tau / (K * L);
Ki_ZN_PID = Kp_ZN_PID / (2 * L);
Kd_ZN_PID = Kp_ZN_PID * (0.5 * L);

fprintf('P Control:\n');
fprintf('  Kp = %.3f\n\n', Kp_ZN_P);

fprintf('PI Control:\n');
fprintf('  Kp = %.3f\n', Kp_ZN_PI);
fprintf('  Ki = %.3f\n\n', Ki_ZN_PI);

fprintf('PID Control:\n');
fprintf('  Kp = %.3f\n', Kp_ZN_PID);
fprintf('  Ki = %.3f\n', Ki_ZN_PID);
fprintf('  Kd = %.3f\n\n', Kd_ZN_PID);

%% Method 2: Cohen-Coon Method

fprintf('=== Cohen-Coon Method ===\n');

% Cohen-Coon formulas (also assumes dead time)
% PI Controller
Kp_CC_PI = (tau / (K * L)) * (0.9 + L / (12 * tau));
Ki_CC_PI = Kp_CC_PI * (30 + 3*L/tau) / (9 + 20*L/tau) / L;

% PID Controller  
Kp_CC_PID = (tau / (K * L)) * (4/3 + L / (4 * tau));
Ki_CC_PID = Kp_CC_PID * (32 + 6*L/tau) / (13 + 8*L/tau) / L;
Kd_CC_PID = Kp_CC_PID * 4 * L / (11 + 2*L/tau);

fprintf('PI Control:\n');
fprintf('  Kp = %.3f\n', Kp_CC_PI);
fprintf('  Ki = %.3f\n\n', Ki_CC_PI);

fprintf('PID Control:\n');
fprintf('  Kp = %.3f\n', Kp_CC_PID);
fprintf('  Ki = %.3f\n', Ki_CC_PID);
fprintf('  Kd = %.3f\n\n', Kd_CC_PID);

%% Method 3: AMIGO Method (Recommended for First-Order Systems)

fprintf('=== AMIGO Method (Åström & Hägglund) ===\n');
fprintf('Optimized for first-order systems\n\n');

% AMIGO PI tuning
Kp_AMIGO = (0.2 + 0.45*tau/L) / K;
Ki_AMIGO = Kp_AMIGO * (0.4*L + 0.8*tau) / (L*tau + 0.1*L*tau) * L;

fprintf('PI Control (RECOMMENDED):\n');
fprintf('  Kp = %.3f\n', Kp_AMIGO);
fprintf('  Ki = %.3f\n', Ki_AMIGO);
fprintf('  Kd = 0 (not needed for thermal systems)\n\n');

%% Method 4: Conservative Manual Tuning (Starting Point)

fprintf('=== Conservative Manual Tuning ===\n');
fprintf('Good starting point for experimentation\n\n');

% Very conservative gains for initial testing
Kp_manual = 1.0 / K;
Ki_manual = Kp_manual / (5 * tau);
Kd_manual = 0;

fprintf('PI Control:\n');
fprintf('  Kp = %.3f\n', Kp_manual);
fprintf('  Ki = %.3f\n', Ki_manual);
fprintf('  Kd = %.3f\n\n', Kd_manual);

%% Simulate Closed-Loop Response

fprintf('=== Simulating Closed-Loop Responses ===\n\n');

% Setpoint step (e.g., 25°C to 50°C)
setpoint = 50;
T0 = 25;

% Time vector
t = 0:0.1:600;  % 10 minutes

% Controllers to compare
controllers = {
    'Z-N PI', Kp_ZN_PI, Ki_ZN_PI, 0;
    'Cohen-Coon PI', Kp_CC_PI, Ki_CC_PI, 0;
    'AMIGO PI', Kp_AMIGO, Ki_AMIGO, 0;
    'Manual PI', Kp_manual, Ki_manual, 0;
};

figure('Position', [100, 100, 1400, 900]);

% Subplot 1: Temperature Response
subplot(2, 2, 1);
hold on;
grid on;

colors = {'r', 'b', 'g', 'm'};
for i = 1:size(controllers, 1)
    name = controllers{i, 1};
    Kp = controllers{i, 2};
    Ki = controllers{i, 3};
    Kd = controllers{i, 4};
    
    % Create PID controller
    C = pid(Kp, Ki, Kd);
    
    % Closed-loop transfer function
    T_cl = feedback(C * G, 1);
    
    % Step response
    [y, t_sim] = step(T_cl, t);
    y = y * (setpoint - T0) + T0;  % Scale and offset
    
    plot(t_sim, y, colors{i}, 'LineWidth', 2, 'DisplayName', name);
    
    % Calculate performance metrics
    info = stepinfo(T_cl);
    fprintf('%s:\n', name);
    fprintf('  Overshoot: %.1f%%\n', info.Overshoot);
    fprintf('  Rise Time: %.1f s\n', info.RiseTime * (setpoint-T0));
    fprintf('  Settling Time: %.1f s\n\n', info.SettlingTime * (setpoint-T0));
end

yline(setpoint, 'k--', 'Setpoint', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Temperature (°C)');
title('Closed-Loop Step Response Comparison');
legend('Location', 'southeast');
ylim([T0-5, setpoint+10]);

% Subplot 2: Control Effort
subplot(2, 2, 2);
hold on;
grid on;

for i = 1:size(controllers, 1)
    name = controllers{i, 1};
    Kp = controllers{i, 2};
    Ki = controllers{i, 3};
    Kd = controllers{i, 4};
    
    C = pid(Kp, Ki, Kd);
    
    % Control effort
    T_err = feedback(1, G*C);  % Error transfer function
    [e, ~] = step(T_err, t);
    u = Kp*e + Ki*cumtrapz(t, e);  % Approximate control signal
    
    plot(t, u, colors{i}, 'LineWidth', 1.5, 'DisplayName', name);
end

xlabel('Time (s)');
ylabel('Control Signal (0-255)');
title('Control Effort (Heater Power)');
legend('Location', 'northeast');
ylim([0, 300]);

% Subplot 3: Error over Time
subplot(2, 2, 3);
hold on;
grid on;

for i = 1:size(controllers, 1)
    name = controllers{i, 1};
    Kp = controllers{i, 2};
    Ki = controllers{i, 3};
    Kd = controllers{i, 4};
    
    C = pid(Kp, Ki, Kd);
    T_cl = feedback(C * G, 1);
    
    [y, ~] = step(T_cl, t);
    error = 1 - y;  % Normalized error
    
    plot(t, error * (setpoint - T0), colors{i}, 'LineWidth', 1.5, 'DisplayName', name);
end

yline(0, 'k--', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Error (°C)');
title('Tracking Error');
legend('Location', 'northeast');

% Subplot 4: Performance Summary Table
subplot(2, 2, 4);
axis off;

% Create performance table
tableData = cell(size(controllers, 1), 4);
for i = 1:size(controllers, 1)
    Kp = controllers{i, 2};
    Ki = controllers{i, 3};
    Kd = controllers{i, 4};
    
    C = pid(Kp, Ki, Kd);
    T_cl = feedback(C * G, 1);
    info = stepinfo(T_cl);
    
    tableData{i, 1} = controllers{i, 1};
    tableData{i, 2} = sprintf('%.1f%%', info.Overshoot);
    tableData{i, 3} = sprintf('%.0fs', info.RiseTime * (setpoint-T0));
    tableData{i, 4} = sprintf('%.0fs', info.SettlingTime * (setpoint-T0));
end

text(0.1, 0.9, 'Performance Summary', 'FontSize', 14, 'FontWeight', 'bold');
text(0.1, 0.75, sprintf('%-20s %10s %12s %15s', 'Method', 'Overshoot', 'Rise Time', 'Settling Time'), ...
     'FontName', 'FixedWidth', 'FontSize', 10);

for i = 1:size(tableData, 1)
    text(0.1, 0.75 - i*0.12, sprintf('%-20s %10s %12s %15s', ...
         tableData{i, 1}, tableData{i, 2}, tableData{i, 3}, tableData{i, 4}), ...
         'FontName', 'FixedWidth', 'FontSize', 9);
end

sgtitle(sprintf('PID Tuning Comparison | System: G(s) = %.1f/(%.1fs + 1)', K, tau), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% Save Results

saveas(gcf, '../results/pid_tuning_comparison.png');
fprintf('✓ Plot saved to: results/pid_tuning_comparison.png\n\n');

% Save recommended gains
gains_file = '../results/recommended_pid_gains.txt';
fid = fopen(gains_file, 'w');
fprintf(fid, 'Recommended PID Gains\n');
fprintf(fid, '=====================\n\n');
fprintf(fid, 'System: G(s) = %.2f / (%.2fs + 1)\n\n', K, tau);
fprintf(fid, '** RECOMMENDED FOR ARDUINO IMPLEMENTATION **\n');
fprintf(fid, 'AMIGO PI Controller:\n');
fprintf(fid, '  Kp = %.3f\n', Kp_AMIGO);
fprintf(fid, '  Ki = %.3f\n', Ki_AMIGO);
fprintf(fid, '  Kd = 0.000\n\n');
fprintf(fid, 'Conservative Starting Point:\n');
fprintf(fid, '  Kp = %.3f\n', Kp_manual);
fprintf(fid, '  Ki = %.3f\n', Ki_manual);
fprintf(fid, '  Kd = 0.000\n\n');
fprintf(fid, 'Other Methods (for comparison):\n');
fprintf(fid, 'Ziegler-Nichols PI: Kp=%.3f, Ki=%.3f\n', Kp_ZN_PI, Ki_ZN_PI);
fprintf(fid, 'Cohen-Coon PI: Kp=%.3f, Ki=%.3f\n', Kp_CC_PI, Ki_CC_PI);
fclose(fid);

fprintf('✓ Gains saved to: results/recommended_pid_gains.txt\n\n');

%% Recommendations

fprintf('=== RECOMMENDATIONS ===\n\n');
fprintf('For Arduino implementation:\n');
fprintf('1. START with conservative manual gains:\n');
fprintf('   Kp = %.3f, Ki = %.3f, Kd = 0\n\n', Kp_manual, Ki_manual);
fprintf('2. TEST response and gradually increase gains\n\n');
fprintf('3. TARGET AMIGO gains for optimal performance:\n');
fprintf('   Kp = %.3f, Ki = %.3f, Kd = 0\n\n', Kp_AMIGO, Ki_AMIGO);
fprintf('Note: Derivative term (Kd) not recommended for\n');
fprintf('      thermal systems due to noise sensitivity.\n\n');