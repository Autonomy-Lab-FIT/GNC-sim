function run_waypoint_ukf_test()

    start_time = tic;
    trajectory_type = 'figure8';
    config = px4_get_config();
    client = px4_connect(config.ip_address, config.port);
    
    % === FLIGHT SEQUENCE ===
    fprintf('Starting %s trajectory test...\n', trajectory_type);
    px4_enter_offboard_mode(client, config);
    pause(2);
    px4_arm_drone(client, config);
    pause(2);
    
    % === GPS REFERENCE SETUP ===
    telemetry = px4_get_telemetry(client, config);
    config = setup_gps_reference(telemetry, config);
    
    % === TRAJECTORY GENERATION ===
    dt_dyn = 0.02;
    dt_gps = 0.1;
    [waypoints, flight_duration] = generate_trajectory(config, trajectory_type, dt_dyn);
    
    % === UKF INITIALIZATION ===
    ukf_cfg = ukf_config_6state();
    [x_ukf, P_ukf] = initialize_ukf(telemetry, config, ukf_cfg);
    
    % === DATA COLLECTION LOOP ===
    fprintf('Starting the simulation...\n');
    t = 0;
    step = 1;
    
    gps_step_interval = round(dt_gps / dt_dyn);
    
    while t < flight_duration && step <= num_steps
        telemetry = px4_get_telemetry(client, config);
        
        % === TRAJECTORY FOLLOWING ===
        current_target = interpolate_trajectory(waypoints, t);
        px4_send_trajectory(client, current_target(1), current_target(2), ...
                           current_target(3), current_target(4), config);
        
        % === UKF UPDATE ===
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
        
        % === DATA LOGGING ===
        data = log_waypoint_data(data, step, t, x_ukf, telemetry, current_target, config);
        
        if mod(step, 250) == 0
            fprintf('Progress: %.1f seconds\n', t);
        end
        
        t = t + dt_dyn;
        step = step + 1;
        pause(dt_dyn);
    end
    
    % === CLEANUP ===
    px4_initiate_landing(client, config);
    pause(5);
    px4_disarm_drone(client, config);
    
    % Save data
    data.actual_steps = step - 1;
    data.trajectory_type = trajectory_type;
    data.waypoints = waypoints;
    
    timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    filename = sprintf('utils/results/waypoint_ukf_test_%s_%s.mat', trajectory_type, timestamp);
    save(filename, 'data');
    
    fprintf('Data saved to: %s\n', filename);
    fprintf('Used %d/%d allocated steps\n', data.actual_steps, num_steps);
    
    analyze_results(filename);

    end_time = toc;
    disp(['Total time: ' num2str(end_time) ' seconds'])
end















