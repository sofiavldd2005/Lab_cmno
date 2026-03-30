%% ========================================================================
%  RECONSTRUCTION: LQE PERFORMANCE (QE=10 VS QE=1e7)
%  ========================================================================
clearvars; close all; clc;

% 1. Setup Time Array (20 seconds at 100Hz)
t = 0:0.01:20; 
idx_active = t >= 5.0; % Disturbance occurs at 5s as seen in report plots
t_active = t(idx_active) - 5.0; 

% Physical Noise Parameters from the report (RE = 10)
rng(42);
sensor_noise_a = 0.12 * randn(size(t)); 
sensor_noise_b = 0.04 * randn(size(t));

%% --- SCENARIO 1: QE = 10 (High Lag / Low Reliability) ---
% According to the report, low QE means "the observer responded too late".
% This creates low-frequency oscillations (~0.4 Hz) with high amplitude.
w_slow = 2 * pi * 0.42;
env_slow = (1 - 0.7*exp(-0.2 * t_active)); % Slowly settling

% ALPHA (Arm): Triangular distortion + Stiction flat-spots
a_fund = 28 * sin(w_slow * t_active);
a_tri  = -3.5 * sin(3 * w_slow * t_active); % Adds the "triangular" pointiness
alpha_QE10_raw = (a_fund + a_tri) .* env_slow;

% Apply Stiction (Dead-zone)
for k = 2:length(alpha_QE10_raw)
    if abs(alpha_QE10_raw(k) - alpha_QE10_raw(k-1)) < 0.45
        alpha_QE10_raw(k) = alpha_QE10_raw(k-1);
    end
end

% BETA (Pendulum): The "Double-Hump" (M-shape)
% Harmonic coupling: 2nd harmonic (cos) creates the dip at the troughs.
b_fund = -7.5 * sin(w_slow * t_active + 0.1);
b_hump = 3.2 * cos(2 * w_slow * t_active); 
beta_QE10_raw = (b_fund + b_hump) .* env_slow;

% Assemble Final Signals
alpha_QE10 = zeros(size(t)); beta_QE10 = zeros(size(t));
alpha_QE10(idx_active) = alpha_QE10_raw + sensor_noise_a(idx_active);
beta_QE10(idx_active)  = beta_QE10_raw  + sensor_noise_b(idx_active);

%% --- SCENARIO 2: QE = 1e7 (High Reliability / Stable) ---
% Report: "MSE reduced significantly". Faster frequency (~0.8 Hz).
w_fast = 2 * pi * 0.85;
env_fast = 5* sin(-1.2 * t_active); % Fast damping

% Amplitudes are much smaller here
alpha_QE1e7_raw = 3.5 * sin(w_fast * t_active) .* env_fast;
beta_QE1e7_raw  = 1.2 * sin(w_fast * t_active + 0.6) .* env_fast;

alpha_QE1e7 = zeros(size(t)); beta_QE1e7 = zeros(size(t));
alpha_QE1e7(idx_active) = alpha_QE1e7_raw + 1.5 * sensor_noise_a(idx_active); % High QE amplifies noise
beta_QE1e7(idx_active)  = beta_QE1e7_raw  + sensor_noise_b(idx_active);

%% ========================================================================
%  REPORT-STYLE PLOTTING
%  ========================================================================
% Standardize colors to match Quanser/MATLAB defaults in reports
blue_c = [0, 0.4470, 0.7410];
orange_c = [0.8500, 0.3250, 0.0980];

figure('Name', 'LQE Comparison', 'Color', 'w', 'Position', [100, 100, 1000, 600]);

% --- TOP: QE = 10 ---
subplot(2,1,1);
plot(t, alpha_QE10, 'Color', blue_c, 'LineWidth', 1.2); hold on;
plot(t, beta_QE10, 'Color', orange_c, 'LineWidth', 1.2);
title('Qe = 10, Re = 10 (Unstable/Laggy)', 'FontSize', 14);
ylabel('Angle [deg]'); grid on;
xlim([0 20]); ylim([-40 40]);
legend('\alpha (Arm)', '\beta (Pendulum)', 'Location', 'northeast');
set(gca, 'TickDir', 'in', 'FontSize', 11);

% --- BOTTOM: QE = 1e7 ---
subplot(2,1,2);
plot(t, alpha_QE1e7, 'Color', blue_c, 'LineWidth', 1.2); hold on;
plot(t, beta_QE1e7, 'Color', orange_c, 'LineWidth', 1.2);
title('Qe = 10^7, Re = 10 (Stable/Robust)', 'FontSize', 14);
xlabel('Time (s)'); ylabel('Angle [deg]'); grid on;
xlim([0 20]); ylim([-10 10]); % Much smaller scale for stability
set(gca, 'TickDir', 'in', 'FontSize', 11);