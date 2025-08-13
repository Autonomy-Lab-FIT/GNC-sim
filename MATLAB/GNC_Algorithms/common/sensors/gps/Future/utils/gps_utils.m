% GPS Utils Functions - All complex implementation details
% Place in: MATLAB/GNC_Algorithms/common/gps/utils/

function gps_utils()
% Empty main function - just makes subfunctions accessible
end

% ==================== CONVERSION UTILITIES ====================

function validate_conversion_inputs(gps_lat, gps_lon, gps_alt, config, num_args)
    if num_args < 4
        error('GPS→NED conversion requires at least 4 inputs: lat, lon, alt, config');
    end
    if ~isfield(config, 'gps') || ~config.gps.home_set
        error('Home position not set. Use px4_set_home_position() first.');
    end
    if ~isnumeric(gps_lat) || ~isnumeric(gps_lon) || ~isnumeric(gps_alt)
        error('GPS coordinates must be numeric values');
    end
    if abs(gps_lat) > 90 || abs(gps_lon) > 180
        error('Invalid GPS coordinates: lat=%.6f, lon=%.6f', gps_lat, gps_lon);
    end
end

function [north, east, down] = perform_gps_conversion(gps_lat, gps_lon, gps_alt, home_lat, home_lon, home_alt)
    try
        if has_function('geodetic2ned')
            wgs84 = get_cached_wgs84_ellipsoid();
            [north, east, down] = geodetic2ned(gps_lat, gps_lon, gps_alt, ...
                                              home_lat, home_lon, home_alt, wgs84);
        elseif has_function('lla2ned')
            [north, east, down] = lla2ned([gps_lat, gps_lon, gps_alt], ...
                                         [home_lat, home_lon, home_alt]);
        else
            error('GPS→NED conversion requires Mapping Toolbox (geodetic2ned) or Aerospace Toolbox (lla2ned)');
        end
    catch ME
        if contains(ME.message, 'geodetic2ned') || contains(ME.message, 'lla2ned')
            error('GPS→NED conversion failed: %s\nRequired toolbox may not be installed.', ME.message);
        else
            rethrow(ME);
        end
    end
end

function has_func = has_function(func_name)
    has_func = (exist(func_name, 'file') == 2);
end

function wgs84 = get_cached_wgs84_ellipsoid()
    persistent wgs84_ellipsoid;
    if isempty(wgs84_ellipsoid)
        wgs84_ellipsoid = wgs84Ellipsoid('meter');
    end
    wgs84 = wgs84_ellipsoid;
end

function should_validate = should_validate(config)
    should_validate = isfield(config, 'gps') && isfield(config.gps, 'validation') && ...
                     config.gps.validation.enable;
end

% ==================== HOME POSITION UTILITIES ====================

function validate_home_inputs(client, config, method, varargin)
    if nargin < 3
        error('px4_set_home_position requires at least 3 inputs: client, config, method');
    end
    valid_methods = {'current', 'manual', 'config'};
    if ~any(strcmpi(method, valid_methods))
        error('Method must be: ''current'', ''manual'', or ''config''');
    end
    if strcmpi(method, 'manual') && length(varargin) < 3
        error('Manual method requires 3 additional arguments: latitude, longitude, altitude');
    end
end

function home_gps = get_current_gps_data(client, config)
    telemetry = px4_get_telemetry(client, config);
    if isempty(telemetry) || ~isfield(telemetry, 'gps') || isempty(telemetry.gps)
        error('No GPS data available. Cannot set home position using current location.');
    end
    home_gps = telemetry.gps;
end

function home_gps = create_manual_gps_data(lat, lon, alt)
    if ~isnumeric(lat) || ~isnumeric(lon) || ~isnumeric(alt)
        error('Manual coordinates must be numeric values');
    end
    if abs(lat) > 90 || abs(lon) > 180
        error('Invalid manual coordinates: lat=%.6f, lon=%.6f', lat, lon);
    end
    home_gps = struct('latitude_deg', lat, 'longitude_deg', lon, 'altitude_msl_m', alt, ...
                     'eph', NaN, 'satellites_used', NaN, 'fix_type', NaN);
end

function home_gps = get_config_gps_data(config)
    if ~isfield(config.gps, 'default_home') || isempty(config.gps.default_home)
        error('No default home position found in configuration');
    end
    home_gps = config.gps.default_home;
end

% ==================== GPS QUALITY UTILITIES ====================

function quality_ok = handle_gps_quality_validation(home_gps, config)
    quality_ok = validate_gps_quality_basic(home_gps, config);
    if ~quality_ok
        show_detailed_gps_quality_warning(home_gps, config);
        user_response = input('Continue with current GPS quality? (y/n): ', 's');
        quality_ok = strcmpi(user_response, 'y') || strcmpi(user_response, 'yes');
    end
end

function quality_ok = validate_gps_quality_basic(gps_data, config)
    thresholds = get_quality_thresholds(config);
    eph_ok = isnan(gps_data.eph) || gps_data.eph <= thresholds.eph_warning;
    sat_ok = isnan(gps_data.satellites_used) || gps_data.satellites_used >= thresholds.satellites_warning;
    fix_ok = ~thresholds.require_3d_fix || isnan(gps_data.fix_type) || gps_data.fix_type == 3;
    quality_ok = eph_ok && sat_ok && fix_ok;
end

function thresholds = get_quality_thresholds(config)
    if isfield(config, 'gps') && isfield(config.gps, 'quality_thresholds')
        thresholds = config.gps.quality_thresholds;
    else
        thresholds.eph_warning = 2.0;
        thresholds.satellites_warning = 6;
        thresholds.require_3d_fix = true;
    end
end

function show_detailed_gps_quality_warning(gps_data, config)
    fprintf('\nWarning: GPS quality suboptimal for home position reference\n');
    thresholds = get_quality_thresholds(config);
    
    if ~isnan(gps_data.eph) && gps_data.eph > thresholds.eph_warning
        fprintf('  Horizontal accuracy: %.1fm (threshold: <%.1fm)\n', gps_data.eph, thresholds.eph_warning);
    end
    if ~isnan(gps_data.satellites_used) && gps_data.satellites_used < thresholds.satellites_warning
        fprintf('  Satellites used: %d (threshold: >%d)\n', gps_data.satellites_used, thresholds.satellites_warning);
    end
    if ~isnan(gps_data.fix_type) && gps_data.fix_type ~= 3 && thresholds.require_3d_fix
        fprintf('  Fix type: %s (requires: 3D)\n', get_fix_type_string(gps_data.fix_type));
    end
    
    fprintf('\nSimulation-specific suggestions:\n');
    fprintf('  - Wait 30-60 seconds for GPS simulation to stabilize\n');
    fprintf('  - Check if custom Gazebo world has GPS noise parameters\n');
    fprintf('  - Consider if this GPS quality is intentional for your test scenario\n\n');
end

% ==================== FILE OPERATIONS UTILITIES ====================

function log_and_display_home_position(home_gps, method, config)
    try
        save_home_position_to_log(home_gps, method);
        fprintf('Home position logged for session recovery.\n');
    catch ME
        warning('Failed to log home position: %s', ME.message); %#ok
    end
    
    display_home_position_confirmation(home_gps, method, config);
end

function save_home_position_to_log(home_gps, method)
    logs_dir = get_logs_directory();
    if ~exist(logs_dir, 'dir'), mkdir(logs_dir); end
    
    log_file = fullfile(logs_dir, 'home_position_log.txt');
    fid = fopen(log_file, 'w');
    if fid == -1, error('Cannot create home position log file: %s', log_file); end
    
    try
        timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
        fprintf(fid, 'Home Position Log - Last Updated: %s\n\n', timestamp);
        fprintf(fid, 'Latitude: %.9f\n', home_gps.latitude_deg);
        fprintf(fid, 'Longitude: %.9f\n', home_gps.longitude_deg);
        fprintf(fid, 'Altitude: %.1f\n', home_gps.altitude_msl_m);
        fprintf(fid, 'Method: %s\n', method);
        fprintf(fid, 'Timestamp: %s\n', timestamp);
        
        if ~isnan(home_gps.eph)
            fprintf(fid, 'GPS_EPH: %.1f\nGPS_Satellites: %d\nGPS_Fix_Type: %d\n', ...
                    home_gps.eph, home_gps.satellites_used, home_gps.fix_type);
        else
            fprintf(fid, 'GPS_EPH: Manual\nGPS_Satellites: Manual\nGPS_Fix_Type: Manual\n');
        end
        fclose(fid);
    catch ME
        fclose(fid);
        rethrow(ME);
    end
end

function logs_dir = get_logs_directory()
    logs_dir = fullfile(fileparts(fileparts(fileparts(fileparts(mfilename('fullpath'))))), 'logs');
end

function log_file = get_home_log_file_path()
    log_file = fullfile(get_logs_directory(), 'home_position_log.txt');
end

% ==================== DISPLAY UTILITIES ====================

function display_home_position_confirmation(home_gps, method, config)
    fprintf('  Coordinates: %.6f°, %.6f°, %.1fm MSL\n', ...
            home_gps.latitude_deg, home_gps.longitude_deg, home_gps.altitude_msl_m);
    fprintf('  Method: %s\n', method);
    
    if strcmp(method, 'current') && ~isnan(home_gps.eph)
        fprintf('  GPS Quality: EPH=%.1fm, Satellites=%d, Fix=%s\n', ...
                home_gps.eph, home_gps.satellites_used, get_fix_type_string(home_gps.fix_type));
    end
    fprintf('  Timestamp: %s\n', config.gps.home_timestamp);
end

function fix_string = get_fix_type_string(fix_type)
    if isnan(fix_type), fix_string = 'Manual'; return; end
    switch fix_type
        case 0, fix_string = 'No Fix';
        case 1, fix_string = 'Dead Reckoning';
        case 2, fix_string = '2D Fix';
        case 3, fix_string = '3D Fix';
        case 4, fix_string = 'GNSS + Dead Reckoning';
        case 5, fix_string = 'Time Only';
        otherwise, fix_string = sprintf('Unknown (%d)', fix_type);
    end
end

% ==================== VALIDATION UTILITIES ====================

function thresholds = get_validation_thresholds(config)
    if isfield(config, 'gps') && isfield(config.gps, 'validation')
        thresholds = config.gps.validation;
    else
        thresholds.horizontal_threshold = 2.0;
        thresholds.vertical_threshold = 2.0;
    end
end

function px4_ned = get_px4_position_data(client, config)
    try
        telemetry = px4_get_telemetry(client, config);
        if isempty(telemetry) || ~isfield(telemetry, 'local_position') || isempty(telemetry.local_position)
            warning('Cannot validate GPS→NED: No PX4 local position data available');
            px4_ned = [];
            return;
        end
        px4_ned = [telemetry.local_position.x, telemetry.local_position.y, telemetry.local_position.z];
    catch ME
        warning('Cannot validate GPS→NED: Failed to get PX4 telemetry - %s', ME.message); %#ok
        px4_ned = [];
    end
end

function [horizontal_diff, vertical_diff, exceeded] = calculate_coordinate_deviations(our_ned, px4_ned, thresholds)
    ned_difference = our_ned - px4_ned;
    horizontal_diff = norm([ned_difference(1), ned_difference(2)]);
    vertical_diff = abs(ned_difference(3));
    exceeded = horizontal_diff > thresholds.horizontal_threshold || vertical_diff > thresholds.vertical_threshold;
end

function display_validation_warning(our_ned, px4_ned, horizontal_diff, vertical_diff, thresholds)
    fprintf('\n');
    warning('GPS→NED conversion differs from PX4 estimates');
    fprintf('  Our GPS→NED:        [%7.2f, %7.2f, %7.2f]m\n', our_ned);
    fprintf('  PX4 Local Position: [%7.2f, %7.2f, %7.2f]m (sensor fusion: GPS+IMU+barometer)\n', px4_ned);
    fprintf('  Distance:           %.2fm horizontal, %.2fm vertical\n', horizontal_diff, vertical_diff);
    fprintf('  Thresholds:         %.1fm horizontal, %.1fm vertical\n', thresholds.horizontal_threshold, thresholds.vertical_threshold);
    fprintf('\nNote: Our conversion uses GPS-only while PX4 uses multi-sensor fusion.\n');
    fprintf('This may indicate GPS coordinate frame issues or GPS quality problems.\n\n');
end

function debug_mode = is_debug_mode(config)
    debug_mode = isfield(config, 'gps') && isfield(config.gps, 'debug') && config.gps.debug;
end

% ==================== GPS QUALITY MONITORING UTILITIES ====================

function is_valid = validate_monitoring_input(telemetry)
%VALIDATE_MONITORING_INPUT Validate telemetry input for GPS quality monitoring
    is_valid = true;
    
    if ~isfield(telemetry, 'gps') || isempty(telemetry.gps)
        warning('No GPS data in telemetry for quality monitoring');
        is_valid = false;
    end
end

function config = initialize_quality_monitoring(config)
%INITIALIZE_QUALITY_MONITORING Initialize GPS quality monitoring state
    % Initialize GPS quality monitoring if not present
    if ~isfield(config, 'gps')
        config.gps = struct();
    end
    
    if ~isfield(config.gps, 'quality_state')
        config.gps.quality_state = struct();
        config.gps.quality_state.last_state = [];
        config.gps.quality_state.last_timestamp = [];
        config.gps.quality_state.session_log = {};
    end
end

function gps_metrics = extract_gps_quality_metrics(gps_data)
%EXTRACT_GPS_QUALITY_METRICS Extract GPS quality metrics from telemetry
    gps_metrics = struct();
    gps_metrics.eph = gps_data.eph;
    gps_metrics.satellites = gps_data.satellites_used;
    gps_metrics.fix_type = gps_data.fix_type;
    gps_metrics.timestamp = datestr(now, 'HH:MM:SS');
end

function [current_state, quality_good] = assess_current_gps_quality(gps_metrics, config)
%ASSESS_CURRENT_GPS_QUALITY Determine current GPS quality status
    % Get quality thresholds
    if isfield(config.gps, 'quality_thresholds')
        thresholds = config.gps.quality_thresholds;
    else
        % Default thresholds
        thresholds.eph_warning = 2.0;
        thresholds.satellites_warning = 6;
        thresholds.require_3d_fix = true;
    end
    
    % Determine current quality status
    eph_good = isnan(gps_metrics.eph) || gps_metrics.eph <= thresholds.eph_warning;
    sat_good = isnan(gps_metrics.satellites) || gps_metrics.satellites >= thresholds.satellites_warning;
    fix_good = ~thresholds.require_3d_fix || isnan(gps_metrics.fix_type) || gps_metrics.fix_type == 3;
    
    quality_good = eph_good && sat_good && fix_good;
    
    % Replace ternary operator with if-else
    if quality_good
        current_state = 'good';
    else
        current_state = 'poor';
    end
end

function handle_quality_transition(current_state, gps_metrics, config)
%HANDLE_QUALITY_TRANSITION Handle GPS quality state transitions
    % Get thresholds for logging
    if isfield(config.gps, 'quality_thresholds')
        thresholds = config.gps.quality_thresholds;
    else
        thresholds.eph_warning = 2.0;
        thresholds.satellites_warning = 6;
        thresholds.require_3d_fix = true;
    end
    
    % Initialize log_entry variable
    log_entry = [];
    
    if strcmp(current_state, 'poor')
        % Quality degradation
        log_entry = create_quality_log_entry('DEGRADATION', gps_metrics.timestamp, ...
                                           gps_metrics.eph, gps_metrics.satellites, ...
                                           gps_metrics.fix_type, thresholds);
        config.gps.quality_state.session_log{end+1} = log_entry;
        
        fprintf('\nGPS Quality DEGRADATION detected at %s:\n', gps_metrics.timestamp);
        display_quality_metrics(gps_metrics.eph, gps_metrics.satellites, ...
                               gps_metrics.fix_type, thresholds);
        
    elseif strcmp(current_state, 'good')
        % Quality recovery
        log_entry = create_quality_log_entry('RECOVERY', gps_metrics.timestamp, ...
                                           gps_metrics.eph, gps_metrics.satellites, ...
                                           gps_metrics.fix_type, thresholds);
        config.gps.quality_state.session_log{end+1} = log_entry;
        
        fprintf('\nGPS Quality RECOVERY detected at %s:\n', gps_metrics.timestamp);
        display_quality_metrics(gps_metrics.eph, gps_metrics.satellites, ...
                               gps_metrics.fix_type, thresholds);
    end
    
    % Write to log file for session analysis (only if log_entry was created)
    if ~isempty(log_entry)
        write_quality_log_entry(log_entry);
    end
end

function log_entry = create_quality_log_entry(transition_type, timestamp, eph, satellites, fix_type, thresholds)
%CREATE_QUALITY_LOG_ENTRY Create structured log entry for quality transition
    log_entry = struct();
    log_entry.type = transition_type;
    log_entry.timestamp = timestamp;
    log_entry.eph = eph;
    log_entry.satellites = satellites;
    log_entry.fix_type = fix_type;
    log_entry.thresholds = thresholds;
end

function display_quality_metrics(eph, satellites, fix_type, thresholds)
%DISPLAY_QUALITY_METRICS Show GPS quality metrics with threshold comparison
    % EPH (horizontal accuracy)
    if ~isnan(eph)
        if eph <= thresholds.eph_warning
            eph_status = 'good';
        else
            eph_status = 'poor';
        end
        fprintf('  Horizontal accuracy: %.1fm (%s, threshold: %.1fm)\n', eph, eph_status, thresholds.eph_warning);
    end
    
    % Satellite count
    if ~isnan(satellites)
        if satellites >= thresholds.satellites_warning
            sat_status = 'good';
        else
            sat_status = 'poor';
        end
        fprintf('  Satellites used: %d (%s, threshold: >=%d)\n', satellites, sat_status, thresholds.satellites_warning);
    end
    
    % Fix type
    if ~isnan(fix_type)
        fix_string = get_fix_type_string(fix_type);
        if fix_type == 3 || ~thresholds.require_3d_fix
            fix_status = 'good';
        else
            fix_status = 'poor';
        end
        fprintf('  Fix type: %s (%s)\n', fix_string, fix_status);
    end
    
    fprintf('\n');
end

function write_quality_log_entry(log_entry)
%WRITE_QUALITY_LOG_ENTRY Write quality transition to log file
    try
        % Determine log file location
        logs_dir = fullfile(fileparts(fileparts(fileparts(fileparts(mfilename('fullpath'))))), 'logs');
        
        % Create logs directory if it doesn't exist
        if ~exist(logs_dir, 'dir')
            mkdir(logs_dir);
        end
        
        log_file = fullfile(logs_dir, 'gps_quality_log.txt');
        
        % Append to quality log file
        fid = fopen(log_file, 'a');
        if fid == -1
            warning('Cannot write to GPS quality log file: %s', log_file);
            return;
        end
        
        try
            % Create log file header if file is new/empty
            file_info = dir(log_file);
            if file_info.bytes == 0
                fprintf(fid, 'GPS Quality Degradation Log\n\n');
            end
            
            % Write log entry
            if strcmp(log_entry.type, 'DEGRADATION')
                fprintf(fid, '%s - DEGRADATION: ', log_entry.timestamp);
                
                issues = {};
                if ~isnan(log_entry.eph) && log_entry.eph > log_entry.thresholds.eph_warning
                    issues{end+1} = sprintf('EPH: %.1fm (threshold: %.1fm)', log_entry.eph, log_entry.thresholds.eph_warning);
                end
                if ~isnan(log_entry.satellites) && log_entry.satellites < log_entry.thresholds.satellites_warning
                    issues{end+1} = sprintf('Satellites: %d (threshold: %d)', log_entry.satellites, log_entry.thresholds.satellites_warning);
                end
                if ~isnan(log_entry.fix_type) && log_entry.fix_type ~= 3 && log_entry.thresholds.require_3d_fix
                    issues{end+1} = sprintf('Fix: %s (requires: 3D)', get_fix_type_string(log_entry.fix_type));
                end
                
                fprintf(fid, '%s\n', strjoin(issues, ', '));
                
            elseif strcmp(log_entry.type, 'RECOVERY')
                fprintf(fid, '%s - RECOVERY: ', log_entry.timestamp);
                if ~isnan(log_entry.eph)
                    fprintf(fid, 'EPH: %.1fm, ', log_entry.eph);
                end
                if ~isnan(log_entry.satellites)
                    fprintf(fid, 'Satellites: %d', log_entry.satellites);
                end
                fprintf(fid, ' - Quality restored\n');
            end
            
            fclose(fid);
            
        catch ME_inner
            fclose(fid);
            warning('Failed to write GPS quality log: %s', ME_inner.message); %#ok
        end
        
    catch ME
        warning('Failed to write GPS quality log: %s', ME.message); %#ok
    end
end

% ==================== GPS RECOVERY UTILITIES ====================

function validate_log_file_access(log_file)
%VALIDATE_LOG_FILE_ACCESS Check log file accessibility and permissions
    % Check log file accessibility
    if ~exist(log_file, 'file')
        error('Home position not recoverable - log file missing: %s', log_file);
    end
    
    % Check file permissions
    fid = fopen(log_file, 'r');
    if fid == -1
        error('Home position not recoverable - insufficient permissions to read: %s', log_file);
    end
    fclose(fid);
end

function home_data = parse_home_position_log(log_file)
%PARSE_HOME_POSITION_LOG Parse the home position log file and return data
    % Open file for reading
    fid = fopen(log_file, 'r');
    if fid == -1
        error('Cannot open log file for reading: %s', log_file);
    end
    
    try
        home_data = parse_home_log_file(fid);
        fclose(fid);
        
    catch ME
        fclose(fid);
        if contains(ME.message, 'corrupted') || contains(ME.message, 'invalid')
            error('Home position not recoverable - log file corrupted: %s', ME.message);
        else
            error('Home position not recoverable - parsing failed: %s', ME.message);
        end
    end
end

function display_recovery_results(home_data, config)
%DISPLAY_RECOVERY_RESULTS Display recovery information and recommendations
    % Display recovery information
    fprintf('Home position recovered from log:\n');
    fprintf('  Coordinates: %.6f°, %.6f°, %.1fm MSL\n', ...
            home_data.gps_data.latitude_deg, home_data.gps_data.longitude_deg, home_data.gps_data.altitude_msl_m);
    fprintf('  Original method: %s\n', home_data.original_method);
    fprintf('  Original timestamp: %s\n', home_data.original_timestamp);
    
    % Show GPS quality from when home was originally set
    if ~isnan(home_data.gps_data.eph)
        quality_assessment = assess_gps_quality_for_recovery(home_data.gps_data, config);
        fprintf('  Original GPS quality when set: EPH=%.1fm, Satellites=%d, Fix=%s (%s)\n', ...
                home_data.gps_data.eph, home_data.gps_data.satellites_used, ...
                get_fix_type_string(home_data.gps_data.fix_type), quality_assessment);
    else
        fprintf('  Original GPS quality when set: Manual coordinates (no quality data)\n');
    end
    
    % Provide recommendation
    if ~isnan(home_data.gps_data.eph)
        if strcmp(assess_gps_quality_for_recovery(home_data.gps_data, config), 'poor')
            fprintf('\nNote: Consider setting new home position if GPS quality is inadequate for your research.\n');
        else
            fprintf('\nGPS quality from original setting appears adequate for research use.\n');
        end
    end
end

function home_data = parse_home_log_file(fid)
%PARSE_HOME_LOG_FILE Parse the home position log file
    home_data = struct();
    home_data.gps_data = struct();
    
    % Initialize fields
    required_fields = {'Latitude', 'Longitude', 'Altitude', 'Method', 'Timestamp'};
    found_fields = false(size(required_fields));
    
    % Read file line by line
    line_number = 0;
    while ~feof(fid)
        line = fgetl(fid);
        line_number = line_number + 1;
        
        if ischar(line) && contains(line, ':')
            % Split on first colon
            colon_pos = find(line == ':', 1);
            if length(colon_pos) >= 1
                key = strtrim(line(1:colon_pos-1));
                value = strtrim(line(colon_pos+1:end));
                
                % Parse known fields
                switch key
                    case 'Latitude'
                        home_data.gps_data.latitude_deg = str2double(value);
                        found_fields(1) = true;
                        
                    case 'Longitude'
                        home_data.gps_data.longitude_deg = str2double(value);
                        found_fields(2) = true;
                        
                    case 'Altitude'
                        home_data.gps_data.altitude_msl_m = str2double(value);
                        found_fields(3) = true;
                        
                    case 'Method'
                        home_data.original_method = value;
                        found_fields(4) = true;
                        
                    case 'Timestamp'
                        home_data.original_timestamp = value;
                        found_fields(5) = true;
                        
                    case 'GPS_EPH'
                        if strcmp(value, 'Manual')
                            home_data.gps_data.eph = NaN;
                        else
                            home_data.gps_data.eph = str2double(value);
                        end
                        
                    case 'GPS_Satellites'
                        if strcmp(value, 'Manual')
                            home_data.gps_data.satellites_used = NaN;
                        else
                            home_data.gps_data.satellites_used = str2double(value);
                        end
                        
                    case 'GPS_Fix_Type'
                        if strcmp(value, 'Manual')
                            home_data.gps_data.fix_type = NaN;
                        else
                            home_data.gps_data.fix_type = str2double(value);
                        end
                end
            end
        end
    end
    
    % Validate required fields were found
    if ~all(found_fields)
        missing = required_fields(~found_fields);
        error('Log file corrupted - missing required fields: %s', strjoin(missing, ', '));
    end
    
    % Validate coordinate values
    if abs(home_data.gps_data.latitude_deg) > 90 || abs(home_data.gps_data.longitude_deg) > 180
        error('Log file contains invalid GPS coordinates: lat=%.6f, lon=%.6f', ...
              home_data.gps_data.latitude_deg, home_data.gps_data.longitude_deg);
    end
end

function quality_str = assess_gps_quality_for_recovery(gps_data, config)
%ASSESS_GPS_QUALITY_FOR_RECOVERY Assess GPS quality as 'good' or 'poor' for recovery
    % Get quality thresholds
    if isfield(config, 'gps') && isfield(config.gps, 'quality_thresholds')
        thresholds = config.gps.quality_thresholds;
    else
        % Default thresholds
        thresholds.eph_warning = 2.0;
        thresholds.satellites_warning = 6;
        thresholds.require_3d_fix = true;
    end
    
    % Check quality metrics
    eph_ok = isnan(gps_data.eph) || gps_data.eph <= thresholds.eph_warning;
    sat_ok = isnan(gps_data.satellites_used) || gps_data.satellites_used >= thresholds.satellites_warning;
    fix_ok = ~thresholds.require_3d_fix || isnan(gps_data.fix_type) || gps_data.fix_type == 3;
    
    if eph_ok && sat_ok && fix_ok
        quality_str = 'good';
    else
        quality_str = 'poor';
    end
end