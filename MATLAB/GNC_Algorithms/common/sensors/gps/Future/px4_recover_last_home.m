function config = px4_recover_last_home(config)
%PX4_RECOVER_LAST_HOME Recover home position from previous session
%   Restores the last set home position from the persistent log file.

    % Initialize GPS config if missing
    if ~isfield(config, 'gps'), config.gps = struct(); end
    
    % Check log file accessibility and parse
    log_file = get_home_log_file_path();
    validate_log_file_access(log_file);  % Throws error if inaccessible
    home_data = parse_home_position_log(log_file);  % Throws error if corrupted
    
    % Restore home position in config
    config.gps.home_position = home_data.gps_data;
    config.gps.home_set = true;
    config.gps.home_method = 'recovered';
    config.gps.home_timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
    
    % Display recovery information and recommendations
    display_recovery_results(home_data, config);
    
    fprintf('Home position recovery completed.\n\n');
end