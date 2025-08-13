function run_hover_ukf_with_lidar()

    start_time = tic;
    config = px4_get_config();
    client = px4_connect(config.ip_address, config.port);

    % === PARAMETERS ===
    dt_dyn = 0.02; %50 hz
    dt_gps = 0.1;
    flight_duration = 60;
    hover_target = [0, 0, -2, 0];

    % === LIDAR SETUP ===
    lidar_update_interval = round(0.2 / dt_dyn); % Update LiDAR plot every 0.2s (5Hz)
    plot_handles = lidar_plot_setup('figure_name', 'UKF Hover - LiDAR Monitor', ...
                                   'max_range', 12.0);

    % === SETUP ===
    ukf_cfg = ukf_config_6state();
    
    % Pre-allocate data arrays
    num_steps = ceil(flight_duration / dt_dyn);
    data = initialize_arrays(num_steps);

    % === FLIGHT SEQUENCE ===
    fprintf('Starting hover test with LiDAR monitoring...\n');
    px4_enter_offboard_mode(client, config);
    pause(2);
    px4_arm_drone(client, config);
    pause(2);

    % === GPS REFERENCE SETUP ===
    telemetry = px4_get_telemetry(client, config);
    config = setup_gps_reference(telemetry, config);
    

    % === Take-off ===
    px4_send_trajectory(client, hover_target(1), hover_target(2), ...
                       hover_target(3), hover_target(4), config);

    % === UKF INITIALIZATION ===
    [x_ukf, P_ukf] = initialize_ukf(telemetry, config, ukf_cfg);

    % === DATA COLLECTION LOOP ===
    fprintf('Starting the simulation with LiDAR monitoring...\n');
    t = 0;
    step = 1;
    data.commanded_x(step) = hover_target(1);
    data.commanded_y(step) = hover_target(2);
    data.commanded_z(step) = hover_target(3);
    
    gps_step_interval = round(dt_gps / dt_dyn);

    while t < flight_duration && step <= num_steps
        telemetry = px4_get_telemetry(client, config);
        gps_update_due = (mod(step-1, gps_step_interval) == 0) && (step > 1);

        if gps_update_due
            y_meas = extract_gps_measurements(telemetry, config);
            if ~isempty(y_meas)
                % GPS measurement available
                [x_ukf, P_ukf] = UKF(@drone_dynamics_6_states, x_ukf, P_ukf, ukf_cfg.Q, ukf_cfg.R_gps, ...
                                    ukf_cfg.alpha, ukf_cfg.beta, ukf_cfg.kappa, ...
                                    dt_dyn, dt_gps, t, y_meas);
            else
                % Prediction only
                [x_ukf, P_ukf] = UKF(@drone_dynamics_6_states, x_ukf, P_ukf, ukf_cfg.Q, ukf_cfg.R_gps, ...
                                    ukf_cfg.alpha, ukf_cfg.beta, ukf_cfg.kappa, ...
                                    dt_dyn, dt_gps, t, []);
            end
        else
            % Prediction only
            [x_ukf, P_ukf] = UKF(@drone_dynamics_6_states, x_ukf, P_ukf, ukf_cfg.Q, ukf_cfg.R_gps, ...
                                ukf_cfg.alpha, ukf_cfg.beta, ukf_cfg.kappa, ...
                                dt_dyn, dt_gps, t, []);
        end

        data = log_data(data, step, t, x_ukf, telemetry, config);

        % === LIDAR UPDATE ===
        lidar_update_due = (mod(step-1, lidar_update_interval) == 0);
        if lidar_update_due && ~isempty(plot_handles)
            try
                lidar_data = px4_get_lidar_scan(client, config, 'include_raw', false);
                if ~isempty(lidar_data) && isfield(lidar_data, 'processed_obstacles')
                    obstacles = lidar_data.processed_obstacles;
                    
                    % Create status with UKF and LiDAR info
                    status_str = sprintf('UKF Hover | t=%.1fs | UKF_z=%.2fm | Obstacles=%d', ...
                                        t, x_ukf(3), length(obstacles));
                    
                    lidar_plot_update(plot_handles, obstacles, 'status_text', status_str);
                end
            catch
                % Continue if LiDAR fails - don't break navigation
            end
        end

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
    timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    filename = sprintf('utils/results/hover_ukf_lidar_test_%s.mat', timestamp);
    save(filename, 'data');
    fprintf('Data saved to: %s\n', filename);
    fprintf('Used %d/%d allocated steps\n', data.actual_steps, num_steps);
    
    analyze_results(filename);
    end_time = toc;
    disp(['Total time: ' num2str(end_time - start_time) ' seconds'])
end