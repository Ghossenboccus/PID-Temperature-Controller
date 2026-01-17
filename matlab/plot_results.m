%% Plot Results - Publication Quality Figures
% PID Temperature Controller Project
% Author: Gabriel Hossenboccus
%
% This script generates clean, professional plots for documentation:
%   1. Step response with model fit
%   2. Closed-loop PID performance
%   3. Comparison plots
%
% Usage:
%   - Load your experimental data
%   - Run this script to generate plots
%   - Figures saved in results/ folder

%% Clear workspace
clear all;
close all;
clc;

%% Configuration
SAVE_PLOTS = true;
EXPORT_FORMAT = 'png';  % 'png', 'pdf', or 'eps'
DPI = 300;              % Resolution for PNG

%% Load Data

fprintf('========================================\n');
fprintf('Plot Results - Publication Quality\n');
fprintf('========================================\n\n');

% Check for step response data
if isfile('../results/step_response.csv')
    data = readmatrix('../results/step_response.csv');
    time = data(:, 1) / 1000;  % Convert to seconds
    temp = data(:, 2);
    has_step_data = true;
    fprintf('✓ Step response data loaded\n');
else
    fprintf('⚠ No step response data found\n');
    has_step_data = false;
end

% Check for PID test data
if isfile('../results/pid_test.csv')
    pid_data = readmatrix('../results/pid_test.csv');
    time_pid = pid_data(:, 1) / 1000;
    temp_pid = pid_data(:, 2);
    setpoint_pid = pid_data(:, 3);
    output_pid = pid_data(:, 4);
    has_pid_data = true;
    fprintf('✓ PID test data loaded\n\n');
else
    fprintf('⚠ No PID test data found\n\n');
    has_pid_data = false;
end

%% Plot 1: Step Response (if available)

if has_step_data
    % Calculate model parameters
    T0 = mean(temp(1:5));
    Tinf = mean(temp(end-9:end));
    K = Tinf - T0;
    
    target_temp = T0 + 0.632 * K;
    idx_63 = find(temp >= target_temp, 1);
    tau = time(idx_63);
    
    % Generate model
    t_model = linspace(0, max(time), 500);
    y_model = T0 + K * (1 - exp(-t_model / tau));
    
    % Create figure
    fig1 = figure('Position', [100, 100, 800, 600]);
    set(fig1, 'Color', 'w');
    
    % Plot experimental data
    plot(time, temp, 'b.', 'MarkerSize', 10, 'DisplayName', 'Experimental');
    hold on;
    
    % Plot model
    plot(t_model, y_model, 'r-', 'LineWidth', 2.5, 'DisplayName', 'Model');
    
    % Add reference lines
    yline(T0 + 0.632*K, 'g--', '63.2% (τ)', 'LineWidth', 1.5, 'FontSize', 11);
    yline(Tinf, 'k--', 'Steady State', 'LineWidth', 1.5, 'FontSize', 11);
    
    % Formatting
    grid on;
    xlabel('Time (s)', 'FontSize', 14, 'FontWeight', 'bold');
    ylabel('Temperature (°C)', 'FontSize', 14, 'FontWeight', 'bold');
    title('Open-Loop Step Response', 'FontSize', 16, 'FontWeight', 'bold');
    legend('Location', 'southeast', 'FontSize', 12);
    set(gca, 'FontSize', 12, 'LineWidth', 1.2);
    
    % Add text annotation
    text_str = sprintf('K = %.1f°C\nτ = %.0fs', K, tau);
    annotation('textbox', [0.15, 0.75, 0.2, 0.1], 'String', text_str, ...
               'FontSize', 13, 'BackgroundColor', 'w', 'EdgeColor', 'k', ...
               'LineWidth', 1.5, 'FitBoxToText', 'on');
    
    if SAVE_PLOTS
        filename = sprintf('../results/step_response_plot.%s', EXPORT_FORMAT);
        if strcmp(EXPORT_FORMAT, 'png')
            print(fig1, filename, '-dpng', sprintf('-r%d', DPI));
        else
            print(fig1, filename, sprintf('-d%s', EXPORT_FORMAT));
        end
        fprintf('✓ Step response plot saved\n');
    end
end

%% Plot 2: PID Performance (if available)

if has_pid_data
    fig2 = figure('Position', [200, 100, 1200, 800]);
    set(fig2, 'Color', 'w');
    
    % Subplot 1: Temperature tracking
    subplot(2, 1, 1);
    plot(time_pid, setpoint_pid, 'k--', 'LineWidth', 2, 'DisplayName', 'Setpoint');
    hold on;
    plot(time_pid, temp_pid, 'b-', 'LineWidth', 2, 'DisplayName', 'Actual');
    
    grid on;
    ylabel('Temperature (°C)', 'FontSize', 13, 'FontWeight', 'bold');
    title('PID Temperature Control Performance', 'FontSize', 15, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 11);
    set(gca, 'FontSize', 11, 'LineWidth', 1.2);
    
    % Subplot 2: Control output
    subplot(2, 1, 2);
    plot(time_pid, output_pid, 'r-', 'LineWidth', 2);
    
    grid on;
    xlabel('Time (s)', 'FontSize', 13, 'FontWeight', 'bold');
    ylabel('Control Output (PWM)', 'FontSize', 13, 'FontWeight', 'bold');
    title('Heater Power', 'FontSize', 15, 'FontWeight', 'bold');
    ylim([0, 255]);
    set(gca, 'FontSize', 11, 'LineWidth', 1.2);
    
    if SAVE_PLOTS
        filename = sprintf('../results/pid_performance.%s', EXPORT_FORMAT);
        if strcmp(EXPORT_FORMAT, 'png')
            print(fig2, filename, '-dpng', sprintf('-r%d', DPI));
        else
            print(fig2, filename, sprintf('-d%s', EXPORT_FORMAT));
        end
        fprintf('✓ PID performance plot saved\n');
    end
end

%% Plot 3: Combined Summary (if both datasets available)

if has_step_data && has_pid_data
    fig3 = figure('Position', [300, 100, 1400, 500]);
    set(fig3, 'Color', 'w');
    
    % Open-loop response
    subplot(1, 2, 1);
    plot(time, temp, 'b-', 'LineWidth', 2);
    hold on;
    plot(t_model, y_model, 'r--', 'LineWidth', 2);
    
    grid on;
    xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Temperature (°C)', 'FontSize', 12, 'FontWeight', 'bold');
    title('Open-Loop (No Control)', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Experimental', 'Model', 'FontSize', 10);
    set(gca, 'FontSize', 11);
    
    % Closed-loop response
    subplot(1, 2, 2);
    plot(time_pid, setpoint_pid, 'k--', 'LineWidth', 2);
    hold on;
    plot(time_pid, temp_pid, 'b-', 'LineWidth', 2);
    
    grid on;
    xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Temperature (°C)', 'FontSize', 12, 'FontWeight', 'bold');
    title('Closed-Loop (PID Control)', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Setpoint', 'Actual', 'FontSize', 10);
    set(gca, 'FontSize', 11);
    
    sgtitle('Temperature Control: Open-Loop vs. Closed-Loop', ...
            'FontSize', 16, 'FontWeight', 'bold');
    
    if SAVE_PLOTS
        filename = sprintf('../results/comparison_plot.%s', EXPORT_FORMAT);
        if strcmp(EXPORT_FORMAT, 'png')
            print(fig3, filename, '-dpng', sprintf('-r%d', DPI));
        else
            print(fig3, filename, sprintf('-d%s', EXPORT_FORMAT));
        end
        fprintf('✓ Comparison plot saved\n');
    end
end

fprintf('\n✓ All plots generated successfully!\n');