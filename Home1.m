%% ========================================================================
%  INVERTED PENDULUM CONTROL (LQR & LQE)
%  ========================================================================
clearvars; close all; clc;
format short

% 1. INITIALIZATION & PARAMETERS
load("IP_MODEL.mat"); % Loads A, B, C, D

% Initial conditions [alpha, alpha_dot, beta, beta_dot, int_alpha]
x0 = [0.1 0 0.05 0 0]'; 

% Physical limits (Constraints for Bryson's Method)
alpha_max     = 90 * pi/180; % Max arm angle (rad) 
beta_max      = 15 * pi/180; % Max pendulum angle (rad) 
alpha_dot_max = 10;          % Max arm velocity (rad/s) 
beta_dot_max  = 10;          % Max pendulum velocity (rad/s) 
int_alpha_max = 2;           % Max accumulated integral error 
v_max         = 5;           % Max motor voltage (V)

Time = 2;                    % General simulation time


%% ========================================================================
%  PART 1: OPEN-LOOP SYSTEM ANALYSIS
%  ========================================================================

% --- Eigenvalues (Stability) ---
e = sort(eig(A));
fprintf('Eigenvalues of A: %.4f, %.4f, %.4f, %.4f, %.4f\n', e);

if any(e < 0) && any(e > 0)
    fprintf(' -> O sistema possui valores próprios com sinais opostos (Instabilidade).\n');
else
    fprintf(' -> O sistema não possui valores próprios com sinais opostos.\n');
end

if any(abs(e) < 1e-10) 
    fprintf(' -> Existe um modo integrador (polo em zero) presente.\n');
end

% --- Controllability ---
n = size(A, 1);
C_0 = ctrb(A, B);
Controlability_rank = rank(C_0);
if any(Controlability_rank < n) 
    fprintf('\nO Sistema não é controlável através de feedback\n');
else 
    fprintf('\nO Sistema é controlável através de feedback e tem dimensão %d.\n', Controlability_rank);
end

% --- Observability ---
C_1 = [0,0,1,0,0];
Observ_matrix_rank_1 = rank(obsv(A, C_1));
if any(Observ_matrix_rank_1 < n) 
    fprintf('O Sistema não é observável medindo apenas beta.\n');
else 
    fprintf('O Sistema é observável medindo beta. Tem característica de %d.\n', Observ_matrix_rank_1);
end

C_2 = [1,0,0,0,0;  0,0,1,0,0];
Observ_matrix_rank_2 = rank(obsv(A, C_2));
if any(Observ_matrix_rank_2 < n) 
    fprintf('O Sistema não é observável medindo alpha e beta.\n\n');
else 
    fprintf('O Sistema é observável medindo alpha e beta. Tem característica de %d.\n\n', Observ_matrix_rank_2);
end

% --- Bode Diagram ---
system_state = ss(A, B, C, D);
w = logspace(-2, 4, 500);

figure('Name', 'Bode Diagram of O.L.', 'Color', 'w');
bode(system_state, w);
grid on;
title('Bode Diagram of the Open-Loop System');
% TODO: Add comments on the diagram results here!


%% ========================================================================
%  PART 2: CONTROLLER DESIGN (LQR)
%  ========================================================================

% --- LQR Design using Bryson's Method ---
Qr_bryson = diag([1/(alpha_max^2), 1/(alpha_dot_max^2), 1/(beta_max^2), 1/(beta_dot_max^2), 1/(int_alpha_max^2)]);
Rr_bryson = 1 / (v_max^2);

K = lqr(A, B, Qr_bryson, Rr_bryson); % Calculate primary feedback gain

disp('--- LQR Controller Matrices (Bryson) ---');
disp('Q Matrix:'); disp(Qr_bryson);
disp('R Matrix:'); disp(Rr_bryson);
disp('Feedback Gain K:'); disp(K);


%% ========================================================================
%  PART 3: OBSERVER DESIGN (KALMAN FILTER)
%  ========================================================================

% --- LQE Tuning Setup ---
G = eye(size(A)); 

% Tuning Scalars (Adjust these in the lab!)
q_noise = 10;   % Model uncertainty factor (Increase to trust sensors more)
r_noise = 1;    % Sensor noise factor (Increase to filter noise more)

Qe = eye(size(A)) * q_noise; 
Re = eye(2) * r_noise;       

% Calculate estimator gains
L = lqe(A, G, C, Qe, Re); 

disp('--- LQE Estimator (Kalman Filter) ---');
disp('Estimator Gain L:'); disp(L);

% Controller matrices for Simulink State-Space block mapping
A_c = A - B*K - L*C;
B_c = L;
C_c = -K;
D_c = zeros(size(K,1),size(C,1));

% Send variables to workspace for Simulink
assignin('base','A_c',A_c);
assignin('base','B_c',B_c);
assignin('base','C_c',C_c);
assignin('base','D_c',D_c);


%% ========================================================================
%  PART 4: SIMULATION & RESULTS
%  ========================================================================

disp('--- Running Final Simulink Simulation ---');
% Simulates using the K and L matrices currently in the workspace
out_final = sim('my_macro_2_2025b.slx', Time);

y_final = out_final.y;
t_final = out_final.tout;

% Calculate Settling Time
settling_threshold = 0.05; 
settlingTime_final = Time;  
state_alpha_beta = y_final(:, 1:2); 

for k = 1:length(t_final)
    if all(all(abs(state_alpha_beta(k, :)) < settling_threshold))
        settlingTime_final = t_final(k);
        break;
    end
end

% Plot Output Response
figure('Name', 'System Response - Final Design', 'Color', 'w');
plot(t_final, y_final, 'LineWidth', 1.5);     

set(gca, 'XColor', 'k', 'YColor', 'k', ...
         'XGrid', 'on', 'YGrid', 'on', ...
         'XMinorGrid', 'on', 'YMinorGrid', 'on', ...
         'GridAlpha', 0.15, 'MinorGridAlpha', 0.15, ...
         'Layer', 'top'); 

title(sprintf('Bryson LQR & Tuned LQE Response (Settling Time: %.2fs)', settlingTime_final), ...
      'FontSize', 14, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontWeight', 'bold');
ylabel('Outputs (rad)', 'FontWeight', 'bold');
legend('\alpha', '\beta', 'Location', 'best');