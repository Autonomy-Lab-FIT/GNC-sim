function params = get_x500_params()

    % https://github.com/PX4/PX4-Autopilot/blob/7f569542a2ef8dbc4549f12cf4f5aef6228c7ac0/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500
    
    params.mass = 1.5; % assumed
    params.gravity = 9.81; % m/s^2
    
    params.inertia = diag([0.029, 0.029, 0.055]); % need to verify this

    params.motor_pos = [
     0.13,  0.22;   % Motor 0: Front Right
    -0.13, -0.20;   % Motor 1: Rear Left  
     0.13, -0.22;   % Motor 2: Rear Right
    -0.13,  0.20    % Motor 3: Front Left
    ];

    % Motor constants (KM values from config)
    params.motor_km = [0.05; 0.05; -0.05; -0.05];  % CCW=+, CW=-

    % arm lengths
    params.L_roll = 0.13;
    params.L_pitch = 0.21; % Avg (0.22+0.20)/2

    % Aerodynamic coeffcients
    params.K_drag_linear = 0.1;
    params.K_drag_angular = 0.01;
    
    default_hover_throttle = 0.60; % 60% hover throttle
    
    % maximum thrust per motor
    total_hover_thrust = params.mass * params.gravity;
    params.T_max = total_hover_thrust / (4 * default_hover_throttle);
    
    % maximum torque per motor
    params.Q_max = abs(params.motor_km(1)) * params.T_max;

end