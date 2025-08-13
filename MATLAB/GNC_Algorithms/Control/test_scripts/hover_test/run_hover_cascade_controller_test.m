function run_hover_cascade_controller_test(config, client)
%RUN_CASCADE_CONTROLLER_TEST Complete UKF + Position + Velocity Controller test
%   Integrates 6-state UKF with position and velocity controllers (full cascade)

    start_time = tic;
    % config = px4_get_config();
    % client = px4_connect(config.ip_address, config.port);

    % === PARAMETERS ===
    dt_dyn = 0.01; % 10 Hz
    dt_gps = 0.1;
    flight_duration = 20;
    hover_target = [0, 0, -2, 0];
    
    % === SETUP ===
    ukf_cfg = ukf_config_6state();
    controller_config = get_cascade_controller_config();
    
    % Initialize velocity controller integral states
    vel_integral_state = [0; 0; 0];
    
    % Pre-allocate data arrays
    num_steps = ceil(flight_duration / dt_dyn);
    data = initialize_arrays_control('cascade', num_steps);
    
    % === FLIGHT SEQUENCE ===
    fprintf('Starting cascade controller test...\n');
    
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
    fprintf('Starting cascade control simulation...\n');
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
        
        % Position controller (outer loop)
        [vel_cmd_x, vel_cmd_y, vel_cmd_z] = position_controller([hover_target(1); hover_target(2); hover_target(3)], x_ukf, controller_config);
        desired_vel = [vel_cmd_x; vel_cmd_y; vel_cmd_z];
        
        % Velocity controller (inner loop)
        [accel_cmd_x, accel_cmd_y, accel_cmd_z, vel_integral_state] = velocity_controller(desired_vel, x_ukf, controller_config, vel_integral_state, dt_dyn);
        
        % Send acceleration setpoints
        px4_send_acceleration_setpoint(client, hover_target(1), hover_target(2), hover_target(3), ...
                           vel_cmd_x, vel_cmd_y, vel_cmd_z, accel_cmd_x, accel_cmd_y, accel_cmd_z, hover_target(4), config);
        
        % px4_send_acceleration_setpoint(client, NaN, NaN, NaN, ...           % No position
        %                       NaN, NaN, NaN, ...                    % No velocity  
        %                       accel_cmd_x, accel_cmd_y, accel_cmd_z, ... % Only acceleration
        %                       hover_target(4), config);

        data = log_data_control(data, step, t, x_ukf, telemetry, config, 'cascade', vel_cmd_x, vel_cmd_y, vel_cmd_z, accel_cmd_x, accel_cmd_y, accel_cmd_z);
        
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
    filename = sprintf('results/hover_cascade_control_test_%s.mat', timestamp);
    save(filename, 'data');
    
    fprintf('Data saved to: %s\n', filename);
    fprintf('Used %d/%d allocated steps\n', data.actual_steps, num_steps);

    analyze_control_results(filename, 'cascade');
    end_time = toc(start_time);
    fprintf('Total time: %.1f seconds\n', end_time);
end