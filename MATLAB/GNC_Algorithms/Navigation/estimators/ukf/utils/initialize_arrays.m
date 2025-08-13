function data = initialize_arrays(num_steps)
    % Time vector
    data.time = zeros(num_steps, 1);
    
    % UKF estimates [px, py, pz, vx, vy, vz]
    data.ukf_states = zeros(num_steps, 6);
    
    % GPS measurements in NED frame [px, py, pz, vx, vy, vz] 
    data.gps_ned = zeros(num_steps, 6);
    
    % PX4 EKF2 estimates [px, py, pz, vx, vy, vz]
    data.ekf2_states = zeros(num_steps, 6);
    
    % For analysis
    data.valid_steps = 0;
end