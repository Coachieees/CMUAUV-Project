function x_ref = fcn(t)
    % Reference trajectory: 1m radius circle at 1.5m depth
    % x_ref = [x, y, z, phi, theta, psi, u, v, w, p, q, r, int_z]
    
    % Circle parameters
    R = 1.0;          % Radius of the circle (meters)
    omega = 0.5;      % Angular velocity (rad/s), approx 62 seconds per lap
    
    % 1. Position Reference (Earth Frame)
    % Center the circle at (0, R) so the trajectory starts smoothly at (0, 0)
    x_d = R * sin(omega * t);
    y_d = R - R * cos(omega * t); 
    
    % Smooth depth transition (ramp to 1.5m over 10 seconds to avoid actuator saturation)
    if t < 10.0
        z_d = (1.5 / 10.0) * t;
    else
        z_d = 1.5;
    end
    
    % 2. Orientation Reference
    phi_d = 0.0;
    theta_d = 0.0;
    
    % Keep yaw aligned with the tangent of the circular path
    psi_d = omega * t;
    
    % Wrap yaw angle to [-pi, pi] to prevent wrap-around error accumulation in SDRE
    psi_d = atan2(sin(psi_d), cos(psi_d));
    
    % 3. Velocity Reference (Body Frame)
    u_d = R * omega;  % Constant forward surge speed
    v_d = 0.0;        % Zero sway (no side-slipping)
    w_d = 0.0;        % Zero heave velocity when cruising
    
    p_d = 0.0;
    q_d = 0.0;
    r_d = omega;      % Constant yaw rate to follow the curve
    
    int_z_d = 0.0;
    
    % Assemble the 13x1 reference vector
    x_ref = [x_d; y_d; z_d; phi_d; theta_d; psi_d; u_d; v_d; w_d; p_d; q_d; r_d; int_z_d];
end
