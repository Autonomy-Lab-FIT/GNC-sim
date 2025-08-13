function config = get_controller_config()
%GET_CONTROLLER_CONFIG Configuration parameters for position controller
%   Returns controller configuration with PX4 default parameters
%
%   Output:
%       config - Structure containing controller parameters

    config = struct();
    
    % PX4 default position controller gains
    config.gains_xy = 0.95;     % MPC_XY_P - Horizontal position gain
    config.gains_z = 1.0;       % MPC_Z_P - Vertical position gain
    
    % PX4 default velocity limits
    config.vel_max_xy = 12.0;   % MPC_XY_VEL_MAX - Max horizontal velocity (m/s)
    config.vel_max_z = 3.0;     % MPC_Z_VEL_MAX - Max vertical velocity (m/s)
    
    % Control timing parameters
    config.control_rate = 50;   % Controller update rate (Hz)
    config.hover_duration = 60; % Test duration (seconds)
    
    % Performance thresholds
    config.position_tolerance = 0.5;    % Position tracking tolerance (m)
    config.velocity_tolerance = 0.2;    % Velocity tracking tolerance (m/s)
    
end