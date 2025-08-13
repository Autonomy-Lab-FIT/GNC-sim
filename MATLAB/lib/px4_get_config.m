function config = px4_get_config()
%PX4_GET_CONFIG Returns configuration settings for PX4 control
%   This function contains all configuration parameters and constants
%   used throughout the PX4 control system.

    % Network configuration
    % config.ip_address = '10.154.5.231';
    config.ip_address = get_first_ip_address;
    config.port = 8766;
    config.timeout_connect = 10;
    config.timeout_response = 1.0;
    config.timeout_telemetry = 2.0;
    config.max_response_size = 1024*1024; % 1MB max
    
    % Debug mode
    config.debug_mode = false;
    
    % MAVLink Command IDs
    config.MAV_CMD_COMPONENT_ARM_DISARM = 400;
    config.MAV_CMD_DO_SET_MODE = 176;
    
    % PX4 Mode values
    config.PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;
    config.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1;
    
    % Flight parameters
    config.takeoff_altitude = -2;  % Z is negative up in PX4 NED frame
    config.land_altitude = 0;
    
    % Pattern parameters
    config.square_size = 5;        % 5m x 5m square
    config.circle_radius = 3;      % 3m radius
    config.circle_points = 12;     % Number of points in circle
    config.waypoint_pause = 5;     % Seconds to pause at each waypoint
    config.circle_pause = 2;       % Seconds to pause at each circle point
    
    % GUI parameters
    config.figure_position = [100, 100, 450, 400];  % Increased width for disarm button
    config.panel_position = [0.05, 0.05, 0.9, 0.9];
    
    % ==================== Camera configuration ====================

    config.camera_compress = true;        % Default: no compression
    config.camera_jpeg_quality = 85;      % JPEG quality (1-100)
    config.camera_timeout_ms = 200;       % Frame request timeout
    config.camera_max_frame_rate = 10;    % Max requests per second
    
    % ==================== LIDAR configuration ====================

    config.lidar_filter_invalid = true;
    config.lidar_min_range = 0.3;           % 30cm minimum
    config.lidar_max_range = 15.0;          % 15m maximum 
    config.lidar_subsample_rate = 1;        % All 1080 points
    config.lidar_include_raw = true;        % Include raw scan data
    config.lidar_include_processed = true;  % Include processed obstacles
    config.lidar_invalid_range_value = 31.0; % max_range + 1.0
    config.lidar_timeout = 2.0;             % Response timeout

    % ==================== GPS COORDINATE CONVERSION SETTINGS ====================
    
    % GPS Quality Thresholds
    config.gps.quality_thresholds.eph_warning = 2.0;      % meters - horizontal accuracy warning
    config.gps.quality_thresholds.satellites_warning = 6;  % count - minimum satellites warning  
    config.gps.quality_thresholds.require_3d_fix = true;   % boolean - require 3D GPS fix
    
    % GPS→NED Validation Settings
    config.gps.validation.enable = true;                   % boolean - cross-validate with PX4
    config.gps.validation.horizontal_threshold = 2.0;      % meters - max horizontal deviation
    config.gps.validation.vertical_threshold = 2.0;        % meters - max vertical deviation
    
    % GPS Debug Settings
    config.gps.debug = false;                              % boolean - show validation debug info
    
    % GPS State (Session-only, initialized empty)
    config.gps.home_position = [];                         % Empty until set by px4_set_home_position
    config.gps.home_set = false;                          % boolean - home position set flag
    config.gps.home_method = '';                           % string - method used to set home
    config.gps.home_timestamp = '';                        % string - when home was set
    
    % GPS Quality State (Session-only monitoring)
    config.gps.quality_state.last_state = [];              % 'good'/'poor' - last quality state
    config.gps.quality_state.last_timestamp = [];          % string - last quality check time
    config.gps.quality_state.session_log = {};             % cell array - session quality transitions
    
    % GPS Default Home Position (Optional - for 'config' method)
    % Uncomment and set if you want a default home position for your research:
    % config.gps.default_home.latitude_deg = 47.397743;     % degrees - Zurich coordinates
    % config.gps.default_home.longitude_deg = 8.545594;     % degrees  
    % config.gps.default_home.altitude_msl_m = 488.0;       % meters MSL
    % config.gps.default_home.eph = NaN;                    % No quality data for manual coordinates
    % config.gps.default_home.satellites_used = NaN;
    % config.gps.default_home.fix_type = NaN;
    
end