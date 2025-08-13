function data = initialize_arrays_control(control_type, num_steps)
%INITIALIZE_ARRAYS_CONTROL Pre-allocate arrays for control tests
%   Inputs:
%       control_type - 'position' or 'cascade'
%       num_steps    - Number of time steps to pre-allocate
    
    data = struct();
    
    % Common arrays for all control types
    data.time = NaN(num_steps, 1);
    data.commanded_x = NaN(num_steps, 1);
    data.commanded_y = NaN(num_steps, 1);
    data.commanded_z = NaN(num_steps, 1);
    
    % UKF estimates (common to all)
    data.ukf_pos_x = NaN(num_steps, 1);
    data.ukf_pos_y = NaN(num_steps, 1);
    data.ukf_pos_z = NaN(num_steps, 1);
    data.ukf_vel_x = NaN(num_steps, 1);
    data.ukf_vel_y = NaN(num_steps, 1);
    data.ukf_vel_z = NaN(num_steps, 1);
    
    % Controller outputs (common to all)
    data.vel_cmd_x = NaN(num_steps, 1);
    data.vel_cmd_y = NaN(num_steps, 1);
    data.vel_cmd_z = NaN(num_steps, 1);
    
    % PX4 reference data (common to all)
    data.px4_pos_x = NaN(num_steps, 1);
    data.px4_pos_y = NaN(num_steps, 1);
    data.px4_pos_z = NaN(num_steps, 1);
    
    % Cascade-specific arrays
    if strcmp(control_type, 'cascade')
        data.accel_cmd_x = NaN(num_steps, 1);
        data.accel_cmd_y = NaN(num_steps, 1);
        data.accel_cmd_z = NaN(num_steps, 1);
    elseif strcmp(control_type, 'mpc')
        data.accel_cmd_x = nan(num_steps, 1);
        data.accel_cmd_y = nan(num_steps, 1);
        data.accel_cmd_z = nan(num_steps, 1);
        
        % MPC-specific arrays
        data.mpc_solve_time = nan(num_steps, 1);    % Solve time in ms
        data.mpc_iterations = nan(num_steps, 1);    % IPOPT iterations
        data.mpc_status = cell(num_steps, 1);       % Solver status
    end
    
end