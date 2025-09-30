function u_sat = saturate_control(u, params)
    u_sat = u;
   
    % Total thrust limits (sum of 4 motors)
    thrust_max = 4 * params.T_max;  % Maximum total thrust
    thrust_min = 0.2 * params.m * params.g;  % Minimum safe thrust (10% hover)
    
    % Thrust saturation - prevent zero thrust that causes crashes
    u_sat(1) = max(thrust_min, min(thrust_max, u(1)));
    
    % Roll moment limits: Lroll * 2 * Tmax 
    roll_moment_max = params.L_roll * 2 * params.T_max;
    u_sat(2) = max(-roll_moment_max, min(roll_moment_max, u(2)));
    
    % Pitch moment limits: Lpitch * 2 * Tmax 
    pitch_moment_max = params.L_pitch * 2 * params.T_max;
    u_sat(3) = max(-pitch_moment_max, min(pitch_moment_max, u(3)));
    
    % Yaw moment limits: 4 * Qmax 
    yaw_moment_max = 4 * params.Q_max;
    u_sat(4) = max(-yaw_moment_max, min(yaw_moment_max, u(4)));
    
    % Log saturation events for debugging
    if u(1) ~= u_sat(1)
        %fprintf('Thrust saturated: %.2f -> %.2f N\n', u(1), u_sat(1));
    end
end