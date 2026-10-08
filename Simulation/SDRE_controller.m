function tau_cmd = fcn(x_est, x_ref, M_total, D_L, D_Q, Q, R, W, B)
    coder.extrinsic('care');
    
    % Pre-allocate extrinsic output (removed the 'info' struct)
    P = zeros(13, 13);
    
    % Extract states and force column vectors
    x_est = x_est(:);
    x_ref = x_ref(:);
    
    eta = x_est(1:6);
    nu = x_est(7:12);
    
    % Initialize matrices from workspace parameters
    M = M_total;
    zg = 0.0620; zb = 0.0213; xb = 0.0050;
    
    % Compute Error
    x_err = x_est - x_ref;
    
    % 1. Kinematic Jacobian J(eta)
    phi = eta(4); theta = eta(5); psi = eta(6);
    J11 = [cos(psi)*cos(theta), -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi), sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);
           sin(psi)*cos(theta), cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi), -cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi);
           -sin(theta),         cos(theta)*sin(phi),                            cos(theta)*cos(phi)];
    J22 = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
           0, cos(phi),           -sin(phi);
           0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    J = [J11, zeros(3,3); zeros(3,3), J22];
    
    % 2. Damping Matrix D(nu)
    D_L_safe = abs(diag(diag(D_L)));
    D_Q_safe = abs(diag(diag(D_Q)));
    D_nu = D_L_safe + D_Q_safe * diag(abs(nu));
    
    % 3. State-Dependent Matrices A(x) and B
    A = zeros(13, 13);
    A(1:6, 7:12) = J;
    A(7:12, 7:12) = -M \ D_nu; 
    A(13, 3) = 1; % Integral of depth error
    
    B_mat = zeros(13, 6);
    B_mat(7:12, 1:6) = inv(M);
    
    % 4. Compute Non-linear Restoring Forces (Feedforward)
    g_eta = [(W-B)*sin(theta);
             -(W-B)*cos(theta)*sin(phi);
             -(W-B)*cos(theta)*cos(phi);
             (zg*W - zb*B)*cos(theta)*sin(phi);
             (zg*W - zb*B)*sin(theta) - (xb*B)*cos(theta)*cos(phi);
             (xb*B)*cos(theta)*sin(phi)];
    
    % 5. SDRE Solver
    persistent K_prev
    if isempty(K_prev)
        % Initialize with a safe, empty matrix
        K_prev = zeros(6, 13);
    end
    
    % Solve ARE (Request only P to avoid struct class mismatches)
    P = care(A, B_mat, Q, R);
    
    % If solution is valid (not empty, NaN, or Inf)
    if ~isempty(P) && ~any(isnan(P(:))) && ~any(isinf(P(:)))
        K = R \ (B_mat' * P);
        K_prev = K; % Save as the last known good gain
    else
        K = K_prev; % Fallback to previous safe gain
    end
    
    % Calculate control law
    tau_cmd = -K * x_err + g_eta;
    
    % Final failsafe to prevent Integrator crash
    if any(isnan(tau_cmd)) || any(isinf(tau_cmd))
        tau_cmd = zeros(6,1);
    end
end
