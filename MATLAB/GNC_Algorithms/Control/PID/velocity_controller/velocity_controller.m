function [accel_cmd_x, accel_cmd_y, accel_cmd_z, vel_integral_state] = velocity_controller(desired_vel, ukf_state, vel_controller_config, vel_integral_state, dt)
%VELOCITY_CONTROLLER_6DOF PID velocity controller for 6-state system
%   Implements PX4-compatible velocity controller using UKF state estimates
%   Outputs acceleration setpoints for PX4 attitude controller
%
%   Inputs:
%       desired_vel         - [3x1] Desired velocity [vx, vy, vz] in NED frame (m/s)
%       ukf_state          - [6x1] UKF state [px, py, pz, vx, vy, vz] 
%       vel_controller_config - Configuration structure with PID gains
%       vel_integral_state - [3x1] Integral state [int_x, int_y, int_z]
%       dt                 - Time step (s)
%
%   Outputs:
%       accel_cmd_x        - North acceleration command (m/s²)
%       accel_cmd_y        - East acceleration command (m/s²) 
%       accel_cmd_z        - Down acceleration command (m/s²)

    % Extract UKF velocity estimates
    ukf_vel_x = ukf_state(4);  % North velocity (m/s)
    ukf_vel_y = ukf_state(5);  % East velocity (m/s)
    ukf_vel_z = ukf_state(6);  % Down velocity (m/s)
    
    % Compute velocity errors (desired - actual)
    vel_error_x = desired_vel(1) - ukf_vel_x;
    vel_error_y = desired_vel(2) - ukf_vel_y;
    vel_error_z = desired_vel(3) - ukf_vel_z;
    
    % Update integral states
    vel_integral_state(1) = vel_integral_state(1) + vel_error_x * dt;
    vel_integral_state(2) = vel_integral_state(2) + vel_error_y * dt;
    vel_integral_state(3) = vel_integral_state(3) + vel_error_z * dt;
    
    % Compute velocity error derivatives (simple backward difference)
    persistent prev_vel_error_x prev_vel_error_y prev_vel_error_z
    if isempty(prev_vel_error_x)
        prev_vel_error_x = vel_error_x;
        prev_vel_error_y = vel_error_y;
        prev_vel_error_z = vel_error_z;
    end
    
    vel_error_dot_x = (vel_error_x - prev_vel_error_x) / dt;
    vel_error_dot_y = (vel_error_y - prev_vel_error_y) / dt;
    vel_error_dot_z = (vel_error_z - prev_vel_error_z) / dt;
    
    % Store current errors for next iteration
    prev_vel_error_x = vel_error_x;
    prev_vel_error_y = vel_error_y;
    prev_vel_error_z = vel_error_z;
    
    % PID controller for horizontal axes (X, Y)
    accel_cmd_x = vel_controller_config.kp_vel_xy * vel_error_x + ...
                  vel_controller_config.ki_vel_xy * vel_integral_state(1) + ...
                  vel_controller_config.kd_vel_xy * vel_error_dot_x;
                  
    accel_cmd_y = vel_controller_config.kp_vel_xy * vel_error_y + ...
                  vel_controller_config.ki_vel_xy * vel_integral_state(2) + ...
                  vel_controller_config.kd_vel_xy * vel_error_dot_y;
    
    % PID controller for vertical axis (Z)
    accel_cmd_z = vel_controller_config.kp_vel_z * vel_error_z + ...
                  vel_controller_config.ki_vel_z * vel_integral_state(3) + ...
                  vel_controller_config.kd_vel_z * vel_error_dot_z;
    
end