% ------------------------------
% CMNO 2024/2025 - Alberto Vale
% ------------------------------

% Load state model
load('IP_MODEL.mat')

% Constraints
V_MAX = 5; % Maximum motor voltage
V_MIN = -5; % Minimum motor voltage

ALPHA_MAX = 90 * pi/180; % Maximum angle accepted for motor shaft
BETA_MAX = 15 * pi/180; % Maximum angle before falling
TIME_DELAY = 6; % seconds before start

T = 30; % Time duration of the simulation
Ts = 0.001; % Sampling time

% Regulator parameters
Qr = diag([10,0,1,0,0]); %Weight Matrix for x
Rr = 0.1; %Weight for the input variable
K = lqr(A, B, Qr, Rr); %Calculate feedback gain

% Estimator parameters
G = eye(size(A)); %
Qe = eye(size(A))*10; %Variance of process errors
Re = eye(2); %Variance of measurement errors
L = lqe(A, G, C, Qe, Re); %Calculate estimator gains


A_init = A;
B_init = B;
C_init = C;
D_int = D;


A = [A_init, zeros(5,1);
    1 0 0 0 0 0];

B = [B_init;
    0];

C = [C_init, zeros(2,1)];

D = zeros(2,1);


Qr = diag([0.4053, 0.01, 14.5903, 0, 0.25, 50]);
Rr = 0.1;
K = lqr(A, B, Qr, Rr);

% Tuning Scalars
q_noise = 10;   
r_noise = 1;    

Qe = eye(size(A_init)) * q_noise; 
Re = eye(2) * r_noise;   
% ---------------------

G5 = eye(5);
L5 = lqe(A_init, G5, C_init, Qe, Re);

% Augmented L: last row integrates measured alpha
L = [L5;
    1 0];
