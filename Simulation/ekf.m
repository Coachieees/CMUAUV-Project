function x_hat = fcn(y_meas, tau_actual, z_ref, M_total, D_L, D_Q, Q_ekf, R_ekf)
    % y_meas: Sensor signals 9x1 [eta(1:6); p; q; r]
    % tau_actual: Control effort 6x1
    y_meas = y_meas(:);
    tau_actual = tau_actual(:);
    
    % Use persistent variables to retain values across time steps
    persistent x_prev P_prev int_z
    if isempty(x_prev)
        x_prev = zeros(12,1); % Initial guess: AUV is stationary
        P_prev = eye(12) * 0.1; 
        int_z = 0;
    end
    
    dt = 0.05; % 20Hz update rate
    M = M_total;
    W = 117.23; B = 119.071; zg = 0.0620; zb = 0.0213; xb = 0.0050;
    
    % ==============================================================
    % 1. PREDICT STEP (Estimate future state using physics model)
    % ==============================================================
    eta = x_prev(1:6);
    nu = x_prev(7:12);
    
    phi = eta(4); theta = eta(5); psi = eta(6);
    cphi = cos(phi); sphi = sin(phi);
    cth  = cos(theta); sth  = sin(theta); tth = tan(theta);
    cpsi = cos(psi); spsi = sin(psi);
    
    % Kinematic Jacobian J(eta)
    J1 = [ cpsi*cth, -spsi*cphi + cpsi*sth*sphi,  spsi*sphi + cpsi*cphi*sth;
           spsi*cth,  cpsi*cphi + sphi*sth*spsi, -cpsi*sphi + sth*spsi*cphi;
          -sth,       cth*sphi,                   cth*cphi ];
    J2 = [ 1, sphi*tth, cphi*tth;
           0, cphi,    -sphi;
           0, sphi/cth, cphi/cth ];
    J = [J1, zeros(3,3); zeros(3,3), J2];
    
    % Calculate Restoring Forces
    g_eta = [(W-B)*sth; -(W-B)*cth*sphi; -(W-B)*cth*cphi;
             (zg*W - zb*B)*cth*sphi; (zg*W - zb*B)*sth - (xb*B)*cth*cphi;
             (xb*B)*cth*sphi];
             
    % Calculate Passive Damping (Matching the plant)
    D_L_safe = abs(diag(diag(D_L)));
    D_Q_safe = abs(diag(diag(D_Q)));
    D_nu = D_L_safe + D_Q_safe * diag(abs(nu));
    
    % Calculate Coriolis (For accurate velocity prediction)
    nu1 = nu(1:3); nu2 = nu(4:6);
    a1 = M(1:3,1:3)*nu1 + M(1:3,4:6)*nu2;
    a2 = M(4:6,1:3)*nu1 + M(4:6,4:6)*nu2;
    S_a1 = [0, -a1(3), a1(2); a1(3), 0, -a1(1); -a1(2), a1(1), 0];
    S_a2 = [0, -a2(3), a2(2); a2(3), 0, -a2(1); -a2(2), a2(1), 0];
    C_nu = [zeros(3,3), -S_a1; -S_a1, -S_a2];
    
    % Nonlinear Prediction
    nu_dot = M \ (tau_actual - C_nu*nu - D_nu*nu - g_eta);
    nu_pred = nu + nu_dot * dt;
    eta_pred = eta + (J * nu) * dt;
    x_pred = [eta_pred; nu_pred];
    
    % Linearized State Transition Matrix (F = I + A*dt)
    A = zeros(12, 12);
    A(1:6, 7:12) = J;
    A(7:12, 7:12) = -M \ D_L_safe; % Use D_L_safe as a proxy for the Jacobian to ensure stability
    F = eye(12) + A * dt;
    
    P_pred = F * P_prev * F' + Q_ekf;
    
    % ==============================================================
    % 2. UPDATE STEP (Correct prediction using actual sensor measurements)
    % ==============================================================
    % H matrix extracts the 9 measured states (6 eta and 3 angular velocities)
    H = zeros(9, 12);
    H(1:6, 1:6) = eye(6);       % Extract position and orientation
    H(7:9, 10:12) = eye(3);     % Extract angular velocities
    
    z_pred = H * x_pred;
    innov = y_meas - z_pred;
    
    % Prevent angle wrap-around errors (Wrap angles to -pi to pi)
    innov(4:6) = atan2(sin(innov(4:6)), cos(innov(4:6)));
    
    S = H * P_pred * H' + R_ekf;
    K = (P_pred * H') / S;
    
    % Update State and Covariance
    x_curr = x_pred + K * innov;
    P_curr = (eye(12) - K * H) * P_pred;
    
    % Save values for the next loop
    x_prev = x_curr;
    P_prev = P_curr;
    
    % ==============================================================
    % 3. ASSEMBLE OUTPUT (Append the 13th state)
    % ==============================================================
    int_z = int_z + (x_curr(3) - z_ref) * dt;
    
    x_hat = [x_curr; int_z];
end
