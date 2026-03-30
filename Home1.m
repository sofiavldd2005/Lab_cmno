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

Time = 10;                    % General simulation time


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
Observ_matrix_rank_1 = rank(obsv(A, C_1))
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

% Find out how many outputs the system has based on the C matrix
num_outputs = size(system_state, 1);

% Loop through each output and plot it in a separate window
for i = 1:num_outputs
    figure('Name', sprintf('Bode Diagram of O.L. - Output %d', i), 'Color', 'w');

    % Plot only the transfer function from Input 1 to Output i
    bode(system_state(i, 1), w);
    grid on;

    title(sprintf('Bode Diagram of the Open-Loop System: Output %d', i));

    % TODO: Add comments on the diagram results here!
end

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
out_final = sim('my_macro_2_2025.slx', Time);

y_final = out_final.y;
t_final = out_final.tout;

% Calculate Settling Time
settling_threshold = 0.01; 
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
 %% Bryson's method & Augmented LQG Controller

        % 1. Maximum acceptable limits (Constraints)
        alpha_max     = 90 * pi/180; 
        beta_max      = 15 * pi/180; 
        alpha_dot_max = 10;          
        beta_dot_max  = 10;          
        current_max   = 5;           
        int_alpha_max = 0.05;
        v_max         = 5;           

        % 2. Calculate Q matrix weights 
        Q11 = 1 / (alpha_max^2);
        Q22 = 1 / (alpha_dot_max^2);
        Q33 = 1 / (beta_max^2);
        Q44 = 1 / (beta_dot_max^2);
        Q55 = 1 / (current_max^2);
        Q66 = 1 / (int_alpha_max^2);
        
        Qr = diag([Q11, Q22, Q33, Q44, Q55, Q66]);
        Rr = 0.04;

        % 3. Create the Augmented PLANT Matrices for LQR (Named Aa, Ba)
        C_alpha = [1, 0, 0, 0, 0]; 
        Aa = [A, zeros(5,1); 
              C_alpha, 0]
        Ba = [B; 
              0]
     
        % 4. Calculate the Augmented Feedback Gain K_aug
        K_aug = lqr(Aa, Ba, Qr, Rr);
        
        
        disp('--- Bryson Method ---');
        disp('Augmented Gain K_aug:');
        disp(K_aug);

        % Extract the primary gains (K) and the integral gain (K_i)
        K   = K_aug(1:5);
        K_i = K_aug(6);

        % ==========================================================
        % OBSERVER (KALMAN FILTER) TUNING
        % ==========================================================
        % The observer only estimates the 5 physical states
        G = eye(size(A)); 

        % Tuning Scalars
        q_noise = 10;   
        r_noise = 1;    

        Qe = eye(size(A)) * q_noise; 
        Re = eye(2) * r_noise;       

        % Calculate estimator gains
        L = lqe(A, G, C, Qe, Re); 

        % ==========================================================
        % LQG CONTROLLER + INTEGRATOR MATRICES FOR SIMULINK
        % ==========================================================
        % We mathematically embed the integrator into the Simulink block!
        
        % The new controller has 6 internal states: 5 from observer, 1 from integrator
        A_c = [A - B*K - L*C,  -B*K_i; 
               0, 0, 0, 0, 0,       0];
        
        % The block takes 'y' as input. y(1) is alpha. 
        % The integrator integrates 1*alpha + 0*beta
        B_c = [L; 
               1, 0]; 
               
        % Output the final control signal u = -Kx - K_i*int
        C_c = -K_aug; 
        
        D_c = [0, 0];

        % Send variables to workspace for Simulink to use
        assignin('base','A_c',A_c);
        assignin('base','B_c',B_c);
        assignin('base','C_c',C_c);
        assignin('base','D_c',D_c);
        

        % ==========================================================
        % SIMULATION & PLOTTING
        % ==========================================================
        
        % Run the Simulink model 
        out_bryson = sim('my_macro_2_2025.slx', Time);

        % Extract data
        y_bryson = out_bryson.y;
        t_bryson = out_bryson.tout;

        % Calculate settling time 
        settlingTime_bryson = Time;  
        state_alpha_beta_bryson = y_bryson(:, 1:2); 
        for k = 1:length(t_bryson)
            if all(all(abs(state_alpha_beta_bryson(k, :)) < settling_threshold))
                settlingTime_bryson = t_bryson(k);
                break;
            end
        end

        % Plot the new response
        figure('Name', 'System Response - Bryson Method (6 States)', 'Color', 'w');
        plot(t_bryson, y_bryson, 'LineWidth', 1.5);     

        % Force the axes to show the grid explicitly
        set(gca, 'XColor', 'k', 'YColor', 'k', ...
            'XGrid', 'on', 'YGrid', 'on', ...
            'XMinorGrid', 'on', 'YMinorGrid', 'on', ...
            'GridAlpha', 0.15, 'MinorGridAlpha', 0.15, ...
            'Layer', 'top'); 

        title(['Bryson Method w/ Integrator (Settling Time: ', num2str(settlingTime_bryson, '%.2f'), 's)'], ...
            'FontSize', 14, 'FontWeight', 'bold');
        xlabel('Time (s)', 'FontWeight', 'bold');
        ylabel('Outputs', 'FontWeight', 'bold');
        legend('\alpha', '\beta', 'Location', 'best');




        