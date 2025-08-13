function run_mpc_point_stabilization_test(config, client)
%RUN_MPC_POINT_STABILIZATION_TEST Complete UKF + MPC Point Stabilization test

    % USER-DEFINED TARGET POSITION
    target_position = [0, 0, -2.5];  % [x, y, z] - modify this if needed
    yaw_target = 0;               % Yaw angle (radians)
    
    start_time = tic;
    
    % Test parameters
    dt_dyn = 0.02;              % 50 Hz control rate
    dt_gps = 0.1;               % 10 Hz GPS updates
    flight_duration = 30;       % 30 second test
    
    % Initialize MPC
    fprintf('Setting up MPC point stabilization test...\n');
    fprintf('Target position: [%.2f, %.2f, %.2f]\n', target_position(1), target_position(2), target_position(3));
    
    mpc_config = get_mpc_config();
    mpc_state = [];  
    
    % UKF setup
    ukf_cfg = ukf_config_6state();
    
    % Pre-allocate data arrays
    num_steps = ceil(flight_duration / dt_dyn);
    data = initialize_arrays_control('mpc', num_steps);
    
    % Flight sequence
    fprintf('Starting MPC point stabilization test...\n');
    px4_enter_offboard_mode(client, config);
    pause(2);
    px4_arm_drone(client, config);
    pause(2);
    
    % GPS reference setup
    telemetry = px4_get_telemetry(client, config);
    config = setup_gps_reference(telemetry, config);
    
    % UKF initialization
    [x_ukf, P_ukf] = initialize_ukf(telemetry, config, ukf_cfg);

    % % Initial takeoff command (use current position initially)
    % current_pos = [telemetry.local_position.x, telemetry.local_position.y, -2];
    % px4_send_trajectory(client, current_pos(1), current_pos(2), current_pos(3), yaw_target, config);
    % 
    % fprintf('Taking off and tracking with UKF...\n');
    % takeoff_duration = 10; % 8 seconds for takeoff
    % takeoff_steps = ceil(takeoff_duration / dt_dyn);
    % t = 0;
    % 
    % for i = 1:takeoff_steps
    %     telemetry = px4_get_telemetry(client, config);
    %     y_meas = extract_gps_measurements(telemetry, config);
    % 
    %     % Always call the UKF to keep the estimate live
    %     [x_ukf, P_ukf] = UKF(@drone_dynamics_6_states, x_ukf, P_ukf, ukf_cfg.Q, ukf_cfg.R_gps, ...
    %                          ukf_cfg.alpha, ukf_cfg.beta, ukf_cfg.kappa, ...
    %                          dt_dyn, dt_gps, t, y_meas);
    % 
    %     fprintf('Tracking... Current Z: %.2f\n', x_ukf(3));
    %     fprintf('Tracking error Z: %.2f\n', target_position(3) - x_ukf(3))
    %     pause(dt_dyn);
    % end

    % Main control loop
    fprintf('Starting MPC control loop...\n');
    t = 0;
    step = 1;
    gps_step_interval = round(dt_gps / dt_dyn);
    
    while t < flight_duration && step <= num_steps
        % Get telemetry
        telemetry = px4_get_telemetry(client, config);
        
        % UKF estimation
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
        
        % fprintf('UKF State - Pos: [%.2f, %.2f, %.2f], Vel: [%.2f, %.2f, %.2f]\n', ...
        %         x_ukf(1), x_ukf(2), x_ukf(3), x_ukf(4), x_ukf(5), x_ukf(6));

        % MPC Point Stabilization Controller
        [pos_cmd, vel_cmd, mpc_state] = mpc_point_stabilization(...
            target_position, x_ukf, mpc_config, mpc_state);
        
        % Send commands to PX4 (guidance-only)
        px4_send_trajectory(client, full(pos_cmd(1)), full(pos_cmd(2)), full(pos_cmd(3)), yaw_target, config);

        % Data logging
        data = log_data_control(data, step, t, x_ukf, telemetry, config, 'mpc', ...
                               full(vel_cmd(1)), full(vel_cmd(2)), full(vel_cmd(3)), ...
                               0, 0, 0); % No acceleration commands in guidance-only
        
        % Progress reporting
        if mod(step, 250) == 0 && isfield(mpc_state, 'solve_time')
            fprintf('Progress: %.1f s, MPC solve time: %.1f ms, Error: %.2f m\n', ...
                    t, mpc_state.solve_time*1000, norm(x_ukf(1:3) - target_position'));
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
    data.target_position = target_position;
    timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    filename = sprintf('results/mpc_point_stabilization_%s.mat', timestamp);
    save(filename, 'data', 'mpc_config');
    
    fprintf('Data saved to: %s\n', filename);
    fprintf('Final position error: %.3f m\n', norm(x_ukf(1:3) - target_position'));
    
    % Analysis
    analyze_control_results(filename, 'mpc');
    
    end_time = toc(start_time);
    fprintf('Total test time: %.1f seconds\n', end_time);
end