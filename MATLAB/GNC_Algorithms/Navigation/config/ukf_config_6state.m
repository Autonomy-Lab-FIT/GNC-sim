function config = ukf_config_6state()
    % UKF parameters (all units meters)
    config.alpha = 0.001;
    config.beta = 2.0;
    config.kappa = 0;

    % === PROCESS NOISE (Account for thrust input) ===
    sigma_pos_xy = 0.1;   
    sigma_pos_z = 0.1;     
    sigma_vel_xy = 15;       
    sigma_vel_z = 15;      

    config.Q = diag([sigma_pos_xy^2, sigma_pos_xy^2, sigma_pos_z^2, ...
                     sigma_vel_xy^2, sigma_vel_xy^2, sigma_vel_z^2]);

    % === GPS MEASUREMENT NOISE ===
    % Based on actual GPS receiver specifications (PINAV)
    gps_pos_std = 5.0;      % 5 m
    gps_vel_std = 0.05;     % 5 cm/s
    
    config.R_gps = diag([gps_pos_std^2, gps_pos_std^2, gps_pos_std^2, ...
                         gps_vel_std^2, gps_vel_std^2, gps_vel_std^2]);

    % === INITIAL COVARIANCE ===
    init_pos_std = 2.0;     
    init_vel_std = 0.1;     
    
    config.P0 = diag([init_pos_std^2, init_pos_std^2, init_pos_std^2, ...
                      init_vel_std^2, init_vel_std^2, init_vel_std^2]);

end