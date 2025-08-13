function config = get_cascade_controller_config()
%GET_CASCADE_CONTROLLER_CONFIG Configuration for position + velocity cascade
%   Returns configuration with PX4 default parameters for both controllers
%
%   Output:
%       config - Structure containing all controller parameters

    config = struct();
    
    % Position controller gains (same as before)
    config.gains_xy = 0.95;     % MPC_XY_P - Horizontal position gain
    config.gains_z = 1.0;       % MPC_Z_P - Vertical position gain
    
    % Position controller velocity limits
    config.vel_max_xy = 12.0;   % MPC_XY_VEL_MAX - Max horizontal velocity (m/s)
    config.vel_max_z = 3.0;     % MPC_Z_VEL_MAX - Max vertical velocity (m/s)
    
    % Velocity controller PID gains - PX4 defaults
    config.kp_vel_xy = 1.8;     % MPC_XY_VEL_P_ACC - Horizontal P gain
    config.ki_vel_xy = 0.4;     % MPC_XY_VEL_I_ACC - Horizontal I gain
    config.kd_vel_xy = 0.2;     % MPC_XY_VEL_D_ACC - Horizontal D gain
    
    config.kp_vel_z = 4.0;      % MPC_Z_VEL_P_ACC - Vertical P gain
    config.ki_vel_z = 2.0;      % MPC_Z_VEL_I_ACC - Vertical I gain
    config.kd_vel_z = 0.0;      % MPC_Z_VEL_D_ACC - Vertical D gain
    
    % Control timing parameters
    config.control_rate = 50;   % Controller update rate (Hz)
    config.hover_duration = 60; % Test duration (seconds)
    
end