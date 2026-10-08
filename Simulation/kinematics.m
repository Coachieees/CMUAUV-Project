function eta_dot = fcn(nu, eta)
    % Extract Euler angles (roll, pitch, yaw)
    phi = eta(4);
    theta = eta(5);
    psi = eta(6);

    % Precompute trigonometric functions to save calculation time
    cphi = cos(phi); sphi = sin(phi);
    cth  = cos(theta); sth  = sin(theta); tth = tan(theta);
    cpsi = cos(psi); spsi = sin(psi);

    % J1: Linear velocity transformation matrix (Body to Earth)
    J1 = [ cpsi*cth, -spsi*cphi + cpsi*sth*sphi,  spsi*sphi + cpsi*cphi*sth;
           spsi*cth,  cpsi*cphi + sphi*sth*spsi, -cpsi*sphi + sth*spsi*cphi;
          -sth,       cth*sphi,                   cth*cphi ];

    % J2: Angular velocity transformation matrix (Body to Earth)
    % Note: A kinematic singularity exists at pitch (theta) = +/- 90 degrees.
    J2 = [ 1, sphi*tth, cphi*tth;
           0, cphi,    -sphi;
           0, sphi/cth, cphi/cth ];

    % Assemble the full 6x6 Jacobian matrix
    J = [J1, zeros(3,3);
         zeros(3,3), J2];

    % Compute the derivative of the earth-fixed states
    eta_dot = J * nu;
end
