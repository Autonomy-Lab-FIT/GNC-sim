function config = setup_gps_reference(telemetry, config)
    %SETUP_GPS_REFERENCE Set GPS reference point from current telemetry
    %   Uses current GPS position as the origin (0,0,0) for NED coordinates
    %
    %   Inputs:
    %       telemetry - Current telemetry from px4_get_telemetry()
    %       config    - Existing configuration structure
    %
    %   Outputs:
    %       config    - Updated config with GPS reference point
    
    % Check if GPS data is available
    if isempty(telemetry) || ~isfield(telemetry, 'gps')
        error('No GPS data available to set reference point');
    end
    
    gps = telemetry.gps;
    
    % Set current GPS position as reference (origin)
    config.ref_lat = gps.latitude_deg;
    config.ref_lon = gps.longitude_deg;
    config.ref_alt = gps.altitude_msl_m;
    
    % Mark reference as set
    config.gps_reference_set = true;
    
    fprintf('GPS reference set to: %.6f°, %.6f°, %.1fm\n', ...
            config.ref_lat, config.ref_lon, config.ref_alt);
    fprintf('This position is now NED origin (0,0,0)\n');
end