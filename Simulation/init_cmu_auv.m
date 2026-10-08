%% CMU-AUV Simulation Initialization Script
clear; clc;

% ==========================================
% 1. Physical Properties & Geometry (Table 1)
% ==========================================
m      = 11.95;                     % Mass [kg]
W      = 117.23;                    % Weight [N]
nabla  = 0.012138;                  % Volume [m^3]
B      = 119.071;                   % Buoyancy [N]
g      = W / m;                     % Gravity [m/s^2] ~ 9.80998
rho    = 1000;                      % Water density [kg/m^3]

% Center of Gravity (CG) and Center of Buoyancy (CB) wrt Origin (CO)
r_g = [0; 0; 0.0620];               % [x_g; y_g; z_g] [m]
r_b = [0.0050; 0; 0.0213];          % [x_b; y_b; z_b] [m]

% Rigid-Body Inertia Tensor at CO [kg*m^2]
Ixx = 0.209849; Iyy = 0.305129; Izz = 0.264812;
Ixy = 0;        Ixz = 0.004125; Iyz = 0;

I_o = [ Ixx, -Ixy, -Ixz;
       -Ixy,  Iyy, -Iyz;
       -Ixz, -Iyz,  Izz ];

% Skew-symmetric matrix of r_g
S_rg = [   0,   -r_g(3),  r_g(2);
         r_g(3),    0,   -r_g(1);
        -r_g(2),  r_g(1),    0   ];

% Rigid-Body Mass Matrix (6x6)
M_RB = [ m * eye(3),    -m * S_rg;
         m * S_rg,        I_o    ];

% ==========================================
% 2. Hydrodynamic Added Mass (Table 3)
% ==========================================
M_A = [  5.55889,   0,       -0.02354,  0,        0.19245,   0;
         0,        13.33090,  0,       -0.78335,  0,         3.04845;
        -0.02972,   0,       15.01354,  0,       -3.43365,   0;
         0,        -0.82329,  0,        0.62734,  0,        -0.18743;
         0.20756,   0,       -3.42374,  0,        0.96326,   0;
         0,         3.05326,  0,       -0.17690,  0,         1.04373 ];

% Total System Inertia Matrix
M_total = M_RB + M_A;
inv_M   = inv(M_total);

% ==========================================
% 3. Damping Coefficients (Tables 4 & 5)
% ==========================================
D_L = [  0,        0,       -5.39181,  0,       -0.02484,  0;
         0,        0,        0,        0.08869,  0,       -0.00479;
         0.27142,  0,        1.13161,  0,       -0.03201,  0;
         0,        0.02142,  0,       -0.01501,  0,       -0.00040;
        -0.09545,  0,       -1.37277,  0,       -0.00294,  0;
         0,       -0.12307,  0,        0.01937,  0,       -0.01100 ];

D_Q = [  1.30135,   0,        55.54980,  0,       -1.17923,  0;
         0,       137.29600,   0,        4.04258,  0,        0.02168;
       125.83700,  -6.90530,   0,        0,        0.53355,  0;
         0,       -10.77595,   0,       -1.17444,  0,        0.02980;
         0,       -28.77614,   5.47058,  0,       -0.90443,  0;
        31.17960,   0,         0,        0.91566,  0,       -1.33297 ];

% ==========================================
% 4. Thruster Configuration Matrix (Table 2)
% ==========================================
% Thruster mapping: tau = TCM * u_thr (u_thr: 8x1 thrust vector)
TCM = [  0.70711,  0.70711, -0.70711, -0.70711,  0.00000,  0.00000,  0.00000,  0.00000;
         0.70711, -0.70711,  0.70711, -0.70711,  0.00000,  0.00000,  0.00000,  0.00000;
         0.00000,  0.00000,  0.00000,  0.00000,  1.00000,  1.00000,  1.00000,  1.00000;
        -0.03111,  0.03111, -0.03111,  0.03111, -0.12000,  0.12000, -0.12000,  0.12000;
         0.03111,  0.03111, -0.03111, -0.03111, -0.05000, -0.05000,  0.05000,  0.05000;
         0.18385, -0.18385, -0.18385,  0.18385,  0.00000,  0.00000,  0.00000,  0.00000 ];

% Moore-Penrose Pseudo-Inverse for Allocation
TCM_pinv = pinv(TCM);

% Thruster limits (adjust according to thruster hardware specs)
T_max = 2.7*9.81;  % [N]
T_min = -2.4*9.81; % [N]

% ==========================================
% 5. SDRE / LQI Tuning Weights
% ==========================================
% State error vector: [e_x, e_y, e_z, e_phi, e_theta, e_psi, u, v, w, p, q, r, int_e_z]
Q_diag = [ 20, 20, 50, ...         % Position weights
           500,  500,  200, ...         % Orientation weights
           20,  20,   40, ...         % Linear velocity damping
           50,  50,   50, ...         % Angular velocity damping
           10 ];                     % Depth integral weight (LQI)
Q = diag(Q_diag);

% Control effort penalty (6-DOF generalized force/torque)
R_diag = [ 10, 10, 10, 50, 50, 50 ];
R = diag(R_diag);

% ==========================================
% 6. EKF Sensor Noise & Filter Covariance
% ==========================================
dt = 0.05; % 50 Hz control loop
Q_ekf = diag([ones(1,6)*1e-4, ones(1,6)*1e-2]);
R_ekf = diag([1e-2, 1e-2, 1e-3, 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2]);
