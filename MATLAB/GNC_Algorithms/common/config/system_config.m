function config = system_config(dt)
%SYSTEM_CONFIG Core system configuration for GNC algorithms
%   Returns fundamental system parameters shared across all algorithms
%
%   Input:
%       dt - Time step in seconds (default: 0.02 = 50Hz)
%
%   Output:
%       config - Structure with core system parameters

    if nargin < 1
        dt = 0.02;  % Default 50Hz update rate
    end
    
    % === TIMING ===
    config.dt = dt;
    config.update_rate = 1/dt;
    
    % === NETWORK CONFIGURATION ===
    config.ip_address = '10.154.5.231';    % Default PX4 bridge IP
    config.port = 8766;                    % Default PX4 bridge port
    config.timeout_connect = 10;           % Connection timeout (seconds)
    config.timeout_response = 1.0;         % Response timeout (seconds)
    config.timeout_telemetry = 2.0;        % Telemetry timeout (seconds)
    config.max_response_size = 1024*1024;  % Maximum response size (1MB)
    config.debug_mode = false;             % Debug output flag
    
    % === STATE VECTOR CONFIGURATION ===
    config.n_states = 13;  % Total state dimension
    
    % State vector indices for clarity
    config.pos_idx = 1:3;       % [x, y, z] position
    config.vel_idx = 4:6;       % [vx, vy, vz] velocity  
    config.quat_idx = 7:10;     % [q0, q1, q2, q3] quaternion
    config.omega_idx = 11:13;   % [wx, wy, wz] angular velocity
    
    % === SENSOR UPDATE RATES ===
    config.gps_update_rate = 10;    % Hz - GPS position/velocity updates
    config.imu_update_rate = 250;   % Hz - IMU gyroscope updates
    config.att_update_rate = 50;    % Hz - Attitude sensor updates
    
    % Calculate update intervals
    config.gps_dt = 1 / config.gps_update_rate;    % GPS update interval
    config.imu_dt = 1 / config.imu_update_rate;    % IMU update interval
    config.att_dt = 1 / config.att_update_rate;    % Attitude update interval
    
    % === MAVLINK COMMAND CONSTANTS ===
    % MAVLink command IDs for vehicle control
    config.MAV_CMD_COMPONENT_ARM_DISARM = 400;
    config.MAV_CMD_DO_SET_MODE = 176;
    
    % PX4 mode values
    config.PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;
    config.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1;
    
    % === FLIGHT CONTROL PARAMETERS ===
    config.takeoff_altitude = -2;  % Z is negative up in PX4 NED frame
    config.land_altitude = 0;
    config.hover_altitude = -2;    % Default hover altitude
    
end