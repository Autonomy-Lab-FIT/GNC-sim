function config = px4_set_home_position(client, config, method, varargin)
%PX4_SET_HOME_POSITION Set GPS home position reference for coordinate conversion
%   Sets the home position reference point for GPS→NED coordinate conversions.
%
%   Usage:
%       config = px4_set_home_position(client, config, 'current')
%       config = px4_set_home_position(client, config, 'manual', lat, lon, alt)
%       config = px4_set_home_position(client, config, 'config')

    % Validate inputs and initialize GPS config
    validate_home_inputs(client, config, method, varargin);
    if ~isfield(config, 'gps'), config.gps = struct(); end
    
    % Get GPS data based on method
    switch lower(method)
        case 'current'
            fprintf('Setting home position using current GPS location...\n');
            home_gps = get_current_gps_data(client, config);
            
        case 'manual'
            fprintf('Setting home position using manual coordinates...\n');
            home_gps = create_manual_gps_data(varargin{1:3});
            
        case 'config'
            fprintf('Setting home position using configuration default...\n');
            home_gps = get_config_gps_data(config);
    end
    
    % GPS quality validation for current method
    if strcmp(method, 'current')
        if ~handle_gps_quality_validation(home_gps, config)
            fprintf('Home position setting cancelled.\n');
            return;
        end
    end
    
    % Set home position in config
    config.gps.home_position = home_gps;
    config.gps.home_set = true;
    config.gps.home_method = method;
    config.gps.home_timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
    
    % Log and display results
    log_and_display_home_position(home_gps, method, config);
    
    fprintf('Home position set successfully.\n');
end