function nu_dot = fcn(tau_actual, nu, eta, M_total, D_Q, D_L)
    % Force column vectors
    tau_actual = tau_actual(:);
    nu = nu(:);
    eta = eta(:);
    
    M = M_total;
    W = 117.23; B = 119.071; zg = 0.0620; zb = 0.0213; xb = 0.0050;
    
    phi = eta(4); theta = eta(5);
    g_eta = [(W-B)*sin(theta); 
             -(W-B)*cos(theta)*sin(phi); 
             -(W-B)*cos(theta)*cos(phi);
             (zg*W - zb*B)*cos(theta)*sin(phi); 
             (zg*W - zb*B)*sin(theta) - (xb*B)*cos(theta)*cos(phi);
             (xb*B)*cos(theta)*sin(phi)];
             
    D_L_safe = abs(diag(diag(D_L)));
    D_Q_safe = abs(diag(diag(D_Q)));
    D_nu = D_L_safe + D_Q_safe * diag(abs(nu));
    
    % =========================================================
    % Fossen's Coriolis and Centripetal Matrix C(nu)
    % =========================================================
    % 1. Split velocities into linear (nu1) and angular (nu2)
    nu1 = nu(1:3);
    nu2 = nu(4:6);
    
    % 2. Split Total Mass Matrix into 3x3 sub-matrices
    M11 = M(1:3, 1:3);
    M12 = M(1:3, 4:6);
    M21 = M(4:6, 1:3);
    M22 = M(4:6, 4:6);
    
    % 3. Calculate terms for the skew-symmetric operator
    a1 = M11 * nu1 + M12 * nu2;
    a2 = M21 * nu1 + M22 * nu2;
    
    % 4. Explicit Skew-symmetric matrices S(a1) and S(a2)
    S_a1 = [   0,   -a1(3),  a1(2); 
             a1(3),    0,   -a1(1); 
            -a1(2),  a1(1),    0   ];
            
    S_a2 = [   0,   -a2(3),  a2(2); 
             a2(3),    0,   -a2(1); 
            -a2(2),  a2(1),    0   ];
            
    % 5. Assemble the full 6x6 C(nu) matrix
    C_nu = [ zeros(3,3),  -S_a1;
               -S_a1,     -S_a2 ];
    % =========================================================
             
    % Full Nonlinear 6-DOF Equations of Motion
    nu_dot = M \ (tau_actual - C_nu * nu - D_nu * nu - g_eta);
end
