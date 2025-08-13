function run_hover_mpc_test(config, client)
%RUN_MPC_HOVER_TEST Complete UKF + MPC Position Controller hover test

    start_time = tic;
    
    % Test parameters
    dt_dyn = 0.02;              % 50 Hz control rate
    dt_gps = 0.1;               % 10 Hz GPS updates
    flight_duration = 30;       % 30 second test
    hover_target = [0, 0, -2, 0]; % [x, y, z, yaw] target
    
    % Initialize MPC
    fprintf('Setting up MPC hover test...\n');
    mpc_config = get_mpc_config();
    mpc_state = [];  
    
    % UKF setup
    ukf_cfg = ukf_config_6state();
    
    % Pre-allocate data arrays
    num_steps = ceil(flight_duration / dt_dyn);
    data = initialize_arrays_control('mpc', num_steps);
    
    % Flight sequence
    fprintf('Starting MPC hover test...\n');
    px4_enter_offboard_mode(client, config);
    pause(2);
    px4_arm_drone(client, config);
    pause(2);
    
    % GPS reference setup
    telemetry = px4_get_telemetry(client, config);
    config = setup_gps_reference(telemetry, config);
    
    % Initial takeoff command
    px4_send_trajectory(client, hover_target(1), hover_target(2), ...
                       hover_target(3), hover_target(4), config);
    pause(8);

    % UKF initialization
    [x_ukf, P_ukf] = initialize_ukf(telemetry, config, ukf_cfg);
    
    % Main control loop
    fprintf('Starting MPC control loop...\n');
    t = 0;
    step = 1;
    gps_step_interval = round(dt_gps / dt_dyn);
    
    while t < flight_duration && step <= num_steps
        % Get telemetry
        telemetry = px4_get_telemetry(client, config);
        
        % UKF estimation (same as cascade controller)
        gps_update_due = (mod(step-1, gps_step_interval) == 0) && (step > 1);
        if gps_update_due
            y_meas = extract_gps_measurements(telemetry, config);
            if ~isempty(y_meas)
                [x_ukf, P_ukf] = UKF(@drone_dynamics_6_states, x_ukf, P_ukf, ukf_cfg.Q, ukf_cfg.R_gps, ...
                                     ukf_cfg.alpha, ukf_cfg.beta, ukf_cfg.kappa, ...
                                     dt_dyn, dt_gps, t, y_meas);
            else
                [x_ukf, P_ukf] = UKF(@drone_dynamics_6_states, x_ukf, P_ukf, ukf_cfg.Q, ukf_cfg.R_gps, ...
                                     ukf_cfg.alpha, ukf_cfg.beta, ukf_cfg.kappa, ...
                                     dt_dyn, dt_gps, t, []);
            end
        else
            [x_ukf, P_ukf] = UKF(@drone_dynamics_6_states, x_ukf, P_ukf, ukf_cfg.Q, ukf_cfg.R_gps, ...
                                 ukf_cfg.alpha, ukf_cfg.beta, ukf_cfg.kappa, ...
                                 dt_dyn, dt_gps, t, []);
        end
        
        % MPC controller
        [pos_cmd, vel_cmd, accel_cmd, mpc_state] = mpc_position_controller(...
            hover_target(1:3), x_ukf, mpc_config, mpc_state);
        
        % Send commands to PX4
        px4_send_trajectory(client, pos_cmd(1), pos_cmd(2), pos_cmd(3), hover_target(4), config);

        % px4_send_acceleration_setpoint(client, 0, 0, 0, 0, 0, 0, ...
        %                       accel_cmd(1), accel_cmd(2), accel_cmd(3), ...
        %                       hover_target(4), config);
        % 

        % Data logging (modify your existing log_data_control function)
        data = log_data_control(data, step, t, x_ukf, telemetry, config, 'mpc', ...
                               vel_cmd(1), vel_cmd(2), vel_cmd(3), ...
                               accel_cmd(1), accel_cmd(2), accel_cmd(3));
        
        % Progress reporting
        if mod(step, 250) == 0 && isfield(mpc_state, 'solve_time')
            fprintf('Progress: %.1f s, MPC solve time: %.1f ms\n', t, mpc_state.solve_time*1000);
        end
        
        t = t + dt_dyn;
        step = step + 1;
        pause(dt_dyn);
    end
    
    % Landing and cleanup
    fprintf('Landing and cleanup...\n');
    px4_initiate_landing(client, config);
    pause(5);
    px4_disarm_drone(client, config);
    
    % Save results
    data.actual_steps = step - 1;
    timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    filename = sprintf('results/hover_mpc_test_%s.mat', timestamp);
    save(filename, 'data', 'mpc_config');
    
    fprintf('Data saved to: %s\n', filename);
    fprintf('Used %d/%d allocated steps\n', data.actual_steps, num_steps);
    
    % Analysis (reuse your existing function)
    analyze_control_results(filename, 'mpc');
    
    end_time = toc(start_time);
    fprintf('Total test time: %.1f seconds\n', end_time);
end