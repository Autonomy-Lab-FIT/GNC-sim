function config = ukf_config()
%UKF_CONFIG Core UKF parameters and noise matrices
%   Returns the essential UKF tuning parameters and noise models
%
%   Output:
%       config - Structure with UKF parameters and noise matrices

    % === CORE UKF PARAMETERS ===
    % These are the three main parameters from your UKF.m
    config.alpha = 0.001;   % Controls spread of sigma points
    config.beta = 2.0;      % Prior knowledge parameter (Gaussian = 2)  
    config.kappa = 0;       % Secondary scaling (typically 0 for n > 3)
    
    % === PROCESS NOISE MATRIX (Q) ===
    % 13x13 matrix for [pos, vel, quat, omega] states
    
    % Individual noise standard deviations (all units in meters and radians)
    sigma_pos = 0.1;        % Position process noise (m)
    sigma_vel = 0.5;        % Velocity process noise (m/s)
    sigma_quat = 0.01;      % Quaternion process noise (rad)
    sigma_omega = 0.1;      % Angular velocity process noise (rad/s)
    
    % Build Q matrix (13x13) - build diagonal step by step
    config.Q = zeros(13, 13);
    config.Q(1:3, 1:3) = sigma_pos^2 * eye(3);         % Position (m²)
    config.Q(4:6, 4:6) = sigma_vel^2 * eye(3);         % Velocity (m²/s²)
    config.Q(7:10, 7:10) = sigma_quat^2 * eye(4);      % Quaternion (rad²)
    config.Q(11:13, 11:13) = sigma_omega^2 * eye(3);   % Angular velocity (rad²/s²)
    
    % === GPS MEASUREMENT NOISE MATRIX (R_gps) ===
    % 6x6 matrix for [position, velocity] measurements
    
    gps_pos_std = 2.0;      % GPS position standard deviation (m)
    gps_vel_std = 0.5;      % GPS velocity standard deviation (m/s)
    
    % Build R_gps matrix (6x6)
    config.R_gps = zeros(6, 6);
    config.R_gps(1:3, 1:3) = gps_pos_std^2 * eye(3);   % Position measurements (m²)
    config.R_gps(4:6, 4:6) = gps_vel_std^2 * eye(3);   % Velocity measurements (m²/s²)
    
    % === IMU MEASUREMENT NOISE MATRIX (R_imu) ===
    % 3x3 matrix for angular velocity measurements
    
    imu_gyro_std = 0.02;    % IMU gyroscope standard deviation (rad/s)
    
    % Build R_imu matrix (3x3)
    config.R_imu = imu_gyro_std^2 * eye(3);             % Angular velocity (rad²/s²)
    
    % === ACCELEROMETER MEASUREMENT NOISE MATRIX (R_accel) ===
    % 3x3 matrix for accelerometer measurements
    
    imu_accel_std = 0.1;    % IMU accelerometer standard deviation (m/s²)
    
    % Build R_accel matrix (3x3)
    config.R_accel = imu_accel_std^2 * eye(3);          % Acceleration (m²/s⁴)
    
    % === VEHICLE PARAMETERS ===
    % Add vehicle parameters that drone_dynamics expects
    config.vehicle_params = get_x500_params();  % Get PX4 X500 parameters
    
    % === INITIAL COVARIANCE MATRIX (P0) ===
    % 13x13 matrix for initial state uncertainty
    
    % Initial uncertainty standard deviations (all units in meters and radians)
    init_pos_std = 1.0;     % Initial position uncertainty (m)
    init_vel_std = 0.5;     % Initial velocity uncertainty (m/s)
    init_quat_std = 0.1;    % Initial quaternion uncertainty (rad)
    init_omega_std = 0.2;   % Initial angular velocity uncertainty (rad/s)
    
    % Build P0 matrix (13x13)
    config.P0 = zeros(13, 13);
    config.P0(1:3, 1:3) = init_pos_std^2 * eye(3);         % Position (m²)
    config.P0(4:6, 4:6) = init_vel_std^2 * eye(3);         % Velocity (m²/s²)
    config.P0(7:10, 7:10) = init_quat_std^2 * eye(4);      % Quaternion (rad²)
    config.P0(11:13, 11:13) = init_omega_std^2 * eye(3);   % Angular velocity (rad²/s²)
    
    % === STORE INDIVIDUAL NOISE PARAMETERS ===
    % Keep individual values for easy tuning (with units specified)
    config.noise_std = struct();
    config.noise_std.position = sigma_pos;                 % (m)
    config.noise_std.velocity = sigma_vel;                 % (m/s)
    config.noise_std.quaternion = sigma_quat;              % (rad)
    config.noise_std.angular_velocity = sigma_omega;       % (rad/s)
    config.noise_std.gps_position = gps_pos_std;           % (m)
    config.noise_std.gps_velocity = gps_vel_std;           % (m/s)
    config.noise_std.imu_gyroscope = imu_gyro_std;         % (rad/s)
    
    % Store initial uncertainty values too (with units specified)
    config.initial_std = struct();
    config.initial_std.position = init_pos_std;            % (m)
    config.initial_std.velocity = init_vel_std;            % (m/s)
    config.initial_std.quaternion = init_quat_std;         % (rad)
    config.initial_std.angular_velocity = init_omega_std;  % (rad/s)
    
end