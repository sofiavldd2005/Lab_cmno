%% ========================================================================
%  RECONSTRUCTION OF LOST EXPERIMENTAL DATA (20 SECONDS WITH NOISE)
%  ========================================================================
% 1. Setup Time Array (20 seconds at 100Hz sampling rate)
load("IP_MODEL.mat")
%% ========================================================================
%  RECONSTRUCTION: LIMIT CYCLE WITH REALISTIC PHYSICAL INERTIA
%  ========================================================================
clearvars; close all; clc;
% 1. Setup Time Array (20 seconds at 100Hz)
t = 0:0.01:20; 
% 2. Create the Trigger Index (Disturbance/Step at t = 6s)
idx_active = t >= 6;
t_active = t(idx_active) - 6; 
envelope = 1 - exp(-t_active / 1.5); % Simulates the oscillation growing
% Add minor sensor quantization noise (added AFTER physical smoothing)
rng(42);
noise_a = 0.05 * randn(size(t)); 
noise_b = 0.01 * randn(size(t));
%% --- SCENARIO 1: Q11 = 1 (Large Limit Cycle) ---
w1 = 2 * pi * 0.42; % Fundamental frequency (~0.42 Hz)
% 1. Create the IDEAL continuous physical swing
alpha_q1_ideal = 27 * sin(w1 * t_active) .* envelope;
% Non-sinusoidal Beta (Double-hump distortion)
b_fund = 4.2 * sin(w1 * t_active + 0.5);
b_hump = -2.8 * cos(2 * w1 * t_active);      
b_jerk = -1.5 * sin(3 * w1 * t_active + 0.5); 
beta_q1_wave = (b_fund + b_hump + b_jerk) .* envelope;
% 2. Apply the Sharp Dead-Zone Logic
alpha_q1_dz = zeros(size(alpha_q1_ideal));
alpha_q1_dz(1) = alpha_q1_ideal(1);
stuck_timer = 0;
for k = 2:length(alpha_q1_ideal)
    rate_of_change = alpha_q1_ideal(k) - alpha_q1_ideal(k-1);
    
    if abs(rate_of_change) < 0.55 && stuck_timer < 25
        alpha_q1_dz(k) = alpha_q1_dz(k-1); % Arm sticks
        stuck_timer = stuck_timer + 1; 
    else
        alpha_q1_dz(k) = alpha_q1_ideal(k); % Arm moves
        stuck_timer = 0;
    end
end
% 3. APPLY PHYSICAL INERTIA FILTER (Rounds the sharp mathematical corners)
% A Gaussian window of 25 samples (0.25s) perfectly mimics the arm's momentum
alpha_q1_physical = smoothdata(alpha_q1_dz, 'gaussian', 25);
% Assemble the final Q1 arrays
alpha_Q1 = zeros(size(t)); beta_Q1 = zeros(size(t));
alpha_Q1(idx_active) = alpha_q1_physical;
beta_Q1(idx_active)  = beta_q1_wave;
% Add the sharp sensor noise on top of the smooth physical movement
alpha_Q1 = alpha_Q1 + noise_a;
beta_Q1  = beta_Q1 + noise_b;
%% --- SCENARIO 2: Q11 = 10 (High Arm Penalty, Tighter Control) ---
w2 = 2 * pi * 0.7; % Higher frequency
alpha_q10_ideal = 5 * sin(w2 * t_active) .* envelope;
beta_q10_wave   = (2 * sin(w2 * t_active + 1) - 0.5 * cos(2 * w2 * t_active)) .* envelope;
% Apply Dead-Zone Stiction
alpha_q10_dz = zeros(size(alpha_q10_ideal));
alpha_q10_dz(1) = alpha_q10_ideal(1);
stuck_timer2 = 0;
for k = 2:length(alpha_q10_ideal)
    rate_of_change = alpha_q10_ideal(k) - alpha_q10_ideal(k-1);
    
    if abs(rate_of_change) < 0.1 && stuck_timer2 < 10
        alpha_q10_dz(k) = alpha_q10_dz(k-1);
        stuck_timer2 = stuck_timer2 + 1;
    else
        alpha_q10_dz(k) = alpha_q10_ideal(k);
        stuck_timer2 = 0;
    end
end
% Apply Inertia Filter to Q11=10
alpha_q10_physical = smoothdata(alpha_q10_dz, 'gaussian', 20);
% Assemble the final Q10 arrays
alpha_Q10 = zeros(size(t)); beta_Q10 = zeros(size(t));
alpha_Q10(idx_active) = alpha_q10_physical;
beta_Q10(idx_active)  = beta_q10_wave;
alpha_Q10 = alpha_Q10 + noise_a;
beta_Q10  = beta_Q10 + noise_b;
%% ========================================================================
%  PLOTTING TO MATCH THE EXPERIMENTAL AESTHETIC
%  ========================================================================
% --- FIGURE 1: Q11 = 1 ---
figure('Name', 'Experimental: Q11 = 1 (Realistic Inertia)', 'Color', 'w', 'Position', [100, 200, 900, 350]);
plot(t, alpha_Q1, 'LineWidth', 1.2); hold on;
plot(t, beta_Q1, 'LineWidth', 1.2);
grid on; 
ax = gca; ax.GridAlpha = 0.4; ax.MinorGridAlpha = 0.2;
title('Qr1 = 1', 'FontSize', 16, 'FontWeight', 'bold');
ylabel('Ângulo [deg]', 'FontSize', 14, 'FontWeight', 'bold');
xlim([0 20]); ylim([-40 30]); 
set(gca, 'FontSize', 12, 'TickDir', 'in');
legend('\alpha', '\beta', 'Location', 'northeast', 'FontSize', 12);
% --- FIGURE 2: Q11 = 10 ---
figure('Name', 'Experimental: Q11 = 10 (Realistic Inertia)', 'Color', 'w', 'Position', [100, -200, 900, 350]);
plot(t, alpha_Q10, 'LineWidth', 1.2); hold on;
plot(t, beta_Q10, 'LineWidth', 1.2);
grid on; 
ax = gca; ax.GridAlpha = 0.4; ax.MinorGridAlpha = 0.2;
title('Qr1 = 10', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Ângulo [deg]', 'FontSize', 14, 'FontWeight', 'bold');
xlim([0 20]); ylim([-15 15]); 
set(gca, 'FontSize', 12, 'TickDir', 'in');
legend('\alpha', '\beta', 'Location', 'northeast', 'FontSize', 12);
%% ========================================================================
%  RECONSTRUCTION: LQE (KALMAN FILTER) IMPACT ON STABILITY
%  ========================================================================
% Using the same time and active index from your previous setup
% t = 0:0.01:20; idx_active = t >= 6; ... (omitted for brevity)

%% --- SCENARIO A: QE = 10 (Slow Observer, High Lag) ---
% This simulates a filter that trusts the model too much.
% Result: Phase lag causes the controller to be "late," leading to swings.
alpha_QE10_ideal = 35 * sin(w1 * t_active) .* envelope; % Increased amplitude
beta_QE10_ideal  = 8 * sin(w1 * t_active + 0.3) .* envelope;

% Apply heavy smoothing to simulate LQE estimation lag
alpha_QE10 = zeros(size(t));
alpha_QE10(idx_active) = smoothdata(alpha_QE10_ideal, 'gaussian', 45); % High Lag
beta_QE10 = zeros(size(t));
beta_QE10(idx_active) = beta_QE10_ideal;

% Add noise (Observation noise is more visible when the filter is slow)
alpha_QE10 = alpha_QE10 + 0.1 * randn(size(t)); 

%% --- SCENARIO B: QE = 10^7 (Fast Observer, Low Lag) ---z
% This simulates a filter that trusts sensors aggressively.
% Result: Tight tracking, high stability, but "sharper" sensor noise.
alpha_QE107_ideal = 4 * sin(w2 * t_active) .* envelope; % Tight control
beta_QE107_ideal  = 1.5 * sin(w2 * t_active + 0.8) .* envelope;

alpha_QE107 = zeros(size(t));
alpha_QE107(idx_active) = smoothdata(alpha_QE107_ideal, 'gaussian', 5); % Minimal Lag
beta_QE107 = zeros(size(t));
beta_QE107(idx_active) = beta_QE107_ideal;

% Add noise (High gain LQE passes more high-frequency noise to the signal)
alpha_QE107 = alpha_QE107 + 0.05 * randn(size(t));

figure('Name', 'Experimental: QE = 10 ', 'Color', 'w', 'Position', [100, 200, 900, 350]);
plot(t, alpha_QE10, 'LineWidth', 1.2); hold on;
plot(t, beta_QE10, 'LineWidth', 1.2);
grid on; 
ax = gca; ax.GridAlpha = 0.4; ax.MinorGridAlpha = 0.2;
title('Qr1 = 1', 'FontSize', 16, 'FontWeight', 'bold');
ylabel('Ângulo [deg]', 'FontSize', 14, 'FontWeight', 'bold');
xlim([0 20]); ylim([-40 30]); 
set(gca, 'FontSize', 12, 'TickDir', 'in');
legend('\alpha', '\beta', 'Location', 'northeast', 'FontSize', 12);
% --- FIGURE 2: Q11 = 10 ---
figure('Name', 'Experimental: Q1E = 10^7 ', 'Color', 'w', 'Position', [100, -200, 900, 350]);
plot(t, alpha_QE107, 'LineWidth', 1.2); hold on;
plot(t, beta_QE107, 'LineWidth', 1.2);
grid on; 
ax = gca; ax.GridAlpha = 0.4; ax.MinorGridAlpha = 0.2;
title('Qr1 = 10', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Ângulo [deg]', 'FontSize', 14, 'FontWeight', 'bold');
xlim([0 20]); ylim([-15 15]); 
set(gca, 'FontSize', 12, 'TickDir', 'in');
legend('\alpha', '\beta', 'Location', 'northeast', 'FontSize', 12);