function mpc_config = get_mpc_config()
%GET_MPC_CONFIGURATION Returns MPC configuration with updated obstacle parameters
    fprintf('Setting up MPC config...\n');
    
    % Timing parameters
    mpc_config.dt = 0.02;           % Control sample time (50 Hz)
    mpc_config.N = 200;             % Prediction horizon, steps

    % Obstacle configuration
    mpc_config.num_sectors = 30;    % Use 30 sectors for 360-degree awareness
    mpc_config.num_obstacles = 10;  % But only use 15 closest obstacles in the solver
    
    mpc_config.guidance_horizon_time = 1.0; % look_ahead, s
    
    % Cost matrix weights
    Q_pos = [1, 1, 10];        % Position weights [x, y, z]
    Q_vel = [0.5, 0.5, 1];    % Velocity weights [vx, vy, vz]
    mpc_config.Q = diag([Q_pos, Q_vel]);
    
    % Control weights
    mpc_config.R = 0.01 * eye(3); % Acceleration weights [ax, ay, az]
    
    % Terminal cost
    mpc_config.Qf = 50 * mpc_config.Q;
    
    % Constraints
    mpc_config.pos_bounds = [-5, 5; -5, 5; -4, 0];     % Position limits [m]
    mpc_config.vel_bounds = [-2, 2; -2, 2; -2, 2];     % Velocity limits [m/s]
    mpc_config.accel_bounds = [-3, 3; -3, 3; -3, 3];   % Acceleration limits [m/s²]
    
    % Set up dynamics model
    mpc_config = mpc_dynamics_model(mpc_config);
end