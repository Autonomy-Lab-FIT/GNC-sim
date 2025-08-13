function y_meas = extract_gps_measurements(telemetry, config)
    %EXTRACT_GPS_MEASUREMENTS Convert GPS data to NED measurements for UKF
    %   
    %   Inputs:
    %       telemetry - Telemetry structure from px4_get_telemetry()
    %       config    - Configuration with GPS reference point
    %
    %   Outputs:
    %       y_meas - 6x1 measurement vector [px, py, pz, vx, vy, vz] in NED
    
    % Check if GPS data is available
    if isempty(telemetry) || ~isfield(telemetry, 'gps')
        error('No GPS data available in telemetry');
    end
    
    % Check if GPS reference is set
    if ~isfield(config, 'ref_lat') || isempty(config.ref_lat)
        error('GPS reference not set in config. Call setup_gps_reference first.');
    end
    
    % Convert GPS lat/lon/alt to NED coordinates
    [north, east, down] = geodetic_to_ned(telemetry.gps.latitude_deg, ...
                                          telemetry.gps.longitude_deg, ...
                                          telemetry.gps.altitude_msl_m, ...
                                          config.ref_lat, ...
                                          config.ref_lon, ...
                                          config.ref_alt);
    %down = -down;
    % GPS velocity (already in NED frame from PX4)
    vel_north = telemetry.gps.vel_n_m_s;
    vel_east = telemetry.gps.vel_e_m_s;
    vel_down = telemetry.gps.vel_d_m_s;
    
    % Assemble measurement vector [position; velocity]
    y_meas = [north; east; down; vel_north; vel_east; vel_down];
    
    % Debug output (optional)
    if isfield(config, 'debug_gps') && config.debug_gps
        fprintf('GPS NED: Pos[%.2f,%.2f,%.2f] Vel[%.2f,%.2f,%.2f]\n', ...
                north, east, down, vel_north, vel_east, vel_down);
    end
end