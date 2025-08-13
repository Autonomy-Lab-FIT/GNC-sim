function config = get_complete_config(dt, custom_overrides)
%GET_COMPLETE_CONFIG Get a complete configuration for UKF estimation
%   Returns a fully configured system by merging system and UKF configurations
%   with optional custom parameter overrides
%
%   Usage:
%       config = get_complete_config();                    % Default 50Hz
%       config = get_complete_config(0.01);               % 100Hz
%       config = get_complete_config(0.02, overrides);    % 50Hz with custom params
%
%   Inputs:
%       dt               - Time step in seconds (optional, default: 0.02)
%       custom_overrides - Structure with custom parameter overrides (optional)
%
%   Output:
%       config - Complete configuration structure with all parameters

    % Set default time step if not provided
    if nargin < 1 || isempty(dt)
        dt = 0.02;  % Default 50Hz
    end
    
    % Validate time step
    if ~isscalar(dt) || dt <= 0
        error('Time step dt must be a positive scalar');
    end
    
    % Get base configurations
    sys_config = system_config(dt);
    ukf_config_result = ukf_config();
    
    % Merge base configurations
    config = merge_config(sys_config, ukf_config_result);
    
    % Apply custom overrides if provided
    if nargin >= 2 && ~isempty(custom_overrides)
        if ~isstruct(custom_overrides)
            error('custom_overrides must be a structure');
        end
        config = merge_config(config, custom_overrides);
    end
    
    % Add configuration metadata
    config.config_info = struct();
    config.config_info.dt = dt;
    config.config_info.frequency = 1/dt;
    config.config_info.has_custom_overrides = (nargin >= 2 && ~isempty(custom_overrides));
    config.config_info.generated_at = datestr(now);
    config.config_info.generated_by = 'get_complete_config';
    
    % Validate the complete configuration
    is_valid = validate_complete_config(config);
    config.config_info.is_valid = is_valid;
    
    if ~is_valid
        warning('Generated configuration may be incomplete or invalid');
    end
    
end

function is_valid = validate_complete_config(config)
%VALIDATE_COMPLETE_CONFIG Check if configuration has all required fields
%   Validates that the complete configuration has all necessary parameters

    is_valid = true;
    
    % Required system fields
    required_system_fields = {'dt', 'n_states', 'pos_idx', 'vel_idx', 'quat_idx', 'omega_idx'};
    
    % Required UKF fields  
    required_ukf_fields = {'alpha', 'beta', 'kappa', 'Q', 'R_gps', 'R_imu', 'P0'};
    
    % Check system fields
    for i = 1:length(required_system_fields)
        field = required_system_fields{i};
        if ~isfield(config, field)
            warning('Missing required system field: %s', field);
            is_valid = false;
        end
    end
    
    % Check UKF fields
    for i = 1:length(required_ukf_fields)
        field = required_ukf_fields{i};
        if ~isfield(config, field)
            warning('Missing required UKF field: %s', field);
            is_valid = false;
        end
    end
    
    % Check matrix dimensions if matrices exist
    if isfield(config, 'Q') && any(size(config.Q) ~= [13, 13])
        warning('Q matrix should be 13x13, got %dx%d', size(config.Q));
        is_valid = false;
    end
    
    if isfield(config, 'R_gps') && any(size(config.R_gps) ~= [6, 6])
        warning('R_gps matrix should be 6x6, got %dx%d', size(config.R_gps));
        is_valid = false;
    end
    
    if isfield(config, 'R_imu') && any(size(config.R_imu) ~= [3, 3])
        warning('R_imu matrix should be 3x3, got %dx%d', size(config.R_imu));
        is_valid = false;
    end
    
    if isfield(config, 'P0') && any(size(config.P0) ~= [13, 13])
        warning('P0 matrix should be 13x13, got %dx%d', size(config.P0));
        is_valid = false;
    end
    
end