function pwm = fcn(u_sat)
    % Convert bounded thrust (Newtons) to PWM signal (1000 - 2000 us)
    pwm = zeros(8,1);
    
    % Max thrust in Surge(forward) requires specific ESC wiring logic:
    % T1, T2, T5, T6 = 1000 | T3, T4 = 2000
    % Assuming standard base mapping where 1500 is stop, we invert specific channels 
    % to match the hardware wiring configuration for the APISQUEEN thrusters.
    
    for i = 1:8
        % Generic linear interpolation (replace with your exact thrust curve)
        if u_sat(i) > 0
            base_pwm = 1500 + (u_sat(i) / 26.487) * 500; 
        else
            base_pwm = 1500 + (u_sat(i) / 23.544) * 500; 
        end
        
        % Apply hardware-specific inversions to match your Surge wiring example
        if i == 1 || i == 2 || i == 5 || i == 6
            pwm(i) = 3000 - base_pwm; % Inverts so positive thrust yields 1000
        else
            pwm(i) = base_pwm; % Positive thrust yields 2000
        end
    end
end
