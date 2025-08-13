function [north, east, down] = test_gps_ned_converison()

    config = px4_get_config();
    client = px4_connect(config.ip_address, config.port);

    % pre_flight_sequence
    px4_run_preflight_sequence(client, config);
    pause(5);
    
    % take-off
    px4_takeoff(client, config);

    % get telemetry
    telemetry = px4_get_telemetry(client, config);

    % set the gps reference(HOME) position the first gps reading
    gps_reference = setup_gps_reference(telemetry);
    [north, east, down] = convert_gps_to_ned(telemetry, gps_reference);
    
    % test positions
    test_positions = [
        0,  0;    % Origin
        5,  0;    % 5m North
        5,  5;    % 5m North, 5m East  
        0,  5;    % 5m East
        0,  0     % Back to origin
    ];

    % comparing with the EKF2 estimater position.
    for i = 1:size(test_positions, 1)
        target_n = test_positions(i, 1);  % North
        target_e = test_positions(i, 2);  % East
        
        fprintf('--- Position %d: Moving to [%dm N, %dm E] ---\n', i, target_n, target_e);
        
        % Move to test position
        px4_send_trajectory(client, target_n, target_e, config.takeoff_altitude, 0, config);
        pause(5);  % Wait to reach position
        
        % Get sensor data at this position
        telemetry = px4_get_telemetry(client, config);
        
        if ~isempty(telemetry) && isfield(telemetry, 'gps') && isfield(telemetry, 'local_position')
            % Current GPS and NED
            curr_lat = telemetry.gps.latitude_deg;
            curr_lon = telemetry.gps.longitude_deg;
            curr_alt = telemetry.gps.altitude_msl_m;
            [gps_north, gps_east, gps_down] = convert_gps_to_ned(telemetry, gps_reference);
            curr_gps = [gps_north; gps_east; gps_down];
            curr_ned = [telemetry.local_position.x; telemetry.local_position.y; telemetry.local_position.z];
            
            % Calculate GPS differences from home
            delta_lat = curr_lat - gps_reference.lat_ref;
            delta_lon = curr_lon - gps_reference.lon_ref;
            delta_alt = curr_alt - gps_reference.alt_ref;
            
            % calculate teh postion error in gps data after convertion with
            % ekf estimate
            delta_pos = curr_gps - curr_ned;

            % Display results
            fprintf('GPS Raw:     %.6f°, %.6f°, %.1fm\n', curr_lat, curr_lon, curr_alt);
            fprintf('GPS Delta:   %.6f°, %.6f°, %.1fm\n', delta_lat, delta_lon, delta_alt);
            fprintf('NED Actual:  [%.2f, %.2f, %.2f]\n', curr_ned(1), curr_ned(2), curr_ned(3));
            fprintf('NED Target:  [%.2f, %.2f, %.2f]\n', target_n, target_e, config.takeoff_altitude);
            fprintf('Position Error:  [%.2f, %.2f, %.2f]\n', delta_pos(1), delta_pos(2), delta_pos(3));
        else
            fprintf('No GPS data available at this position\n\n');
        end
        
        pause(2);
    end
    
    % Land
    fprintf('4. Landing and final verification...\n');
    px4_initiate_landing(client, config);
    
    % Wait for landing
    for j = 1:20
        [landed, ~] = px4_check_landing_status(client, config);
        if landed, break; end
        pause(1);
    end

end