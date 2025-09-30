function run_hover_ukf_test()
    
    start_time = tic;
    config = px4_get_config();
    client = px4_connect(config.ip_address, config.port);

    % parameters
    dt_dyn = 0.02; %50 hz
    dt_gps = 0.1; %10 hz
    flight_duration = 30;

    % target variables
    hover_target = [0, 0, -2, 0];
    
    % ukg config
    ukf_cfg = ukf_config_6state();
    
    % Pre-allocate data arrays
    num_steps = ceil(flight_duration / dt_dyn);
    data = initialize_arrays(num_steps);
    
    % arm the drone
    fprintf('Starting hover test...\n');
    
    px4_enter_offboard_mode(client, config);
    pause(2);
    px4_arm_drone(client, config);
    pause(2);
    
    % Set GPS home, as origin
    telemetry = px4_get_telemetry(client, config); % get telemetry to set the home position
    config = setup_gps_reference(telemetry, config);  % Set GPS reference
    
    
    % take-off
    px4_send_trajectory(client, hover_target(1), hover_target(2), ...
                       hover_target(3), hover_target(4), config);
    
    % initial ukf states and covariance matrix
    [x_ukf, P_ukf] = initialize_ukf(telemetry, config, ukf_cfg);
    
    % simulation loop
    fprintf('Starting the simulation...\n');
    t = 0;
    step = 1;
    
    data.commanded_x(step) = hover_target(1); % 0
    data.commanded_y(step) = hover_target(2); % 0  
    data.commanded_z(step) = hover_target(3); % -2

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
                %fprintf('UKF after update: z=%.3f\n', x_ukf(3));
            else
                % Prediction only
                [x_ukf, P_ukf] = UKF(@drone_dynamics_6_states, x_ukf, P_ukf, ukf_cfg.Q, ukf_cfg.R_gps, ...
                                     ukf_cfg.alpha, ukf_cfg.beta, ukf_cfg.kappa, ...
                                     dt_dyn, dt_gps, t, []);
                %fprintf('UKF after update w/o meas: z=%.3f\n', x_ukf(3));
            end
        else
            % Prediction only
            [x_ukf, P_ukf] = UKF(@drone_dynamics_6_states, x_ukf, P_ukf, ukf_cfg.Q, ukf_cfg.R_gps, ...
                                 ukf_cfg.alpha, ukf_cfg.beta, ukf_cfg.kappa, ...
                                 dt_dyn, dt_gps, t, []);
            %fprintf('UKF after update w/o meas: z=%.3f\n', x_ukf(3));
        end
        data = log_data(data, step, t, x_ukf, telemetry, config);

        if mod(step, 250) == 0
            fprintf('Progress: %.1f seconds\n', t);
        end

        t = t + dt_dyn;
        step = step + 1;
        pause(dt_dyn);
    end

    % Lanfing
    px4_initiate_landing(client, config);
    pause(5);
    px4_disarm_drone(client, config);

    % Save data 
    data.actual_steps = step - 1;  % Store how many steps we used

    timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    filename = sprintf('results/hover_ukf_test_%s.mat', timestamp);
    save(filename, 'data');

    fprintf('Data saved to: %s\n', filename);
    fprintf('Used %d/%d allocated steps\n', data.actual_steps, num_steps);

    analyze_results(filename);
    end_time = toc;
    disp(['Total time: ' num2str(end_time - start_time) ' seconds'])
end
