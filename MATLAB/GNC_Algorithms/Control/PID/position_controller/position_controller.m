function [vel_cmd_x, vel_cmd_y, vel_cmd_z] = position_controller(desired_pos, ukf_state, controller_config)
%POSITION_CONTROLLER P controller for 6-state position control
%   Implements PX4-compatible position controller using UKF state estimates
%
%   Inputs:
%       desired_pos      - [3x1] Desired position [x, y, z] in NED frame (m)
%       ukf_state        - [6x1] UKF state [px, py, pz, vx, vy, vz] 
%       controller_config - Configuration structure with gains and limits
%
%   Outputs:
%       vel_cmd_x        - North velocity command (m/s)
%       vel_cmd_y        - East velocity command (m/s) 
%       vel_cmd_z        - Down velocity command (m/s)

    % Extract UKF position estimates
    ukf_pos_x = ukf_state(1);  % North position (m)
    ukf_pos_y = ukf_state(2);  % East position (m)
    ukf_pos_z = ukf_state(3);  % Down position (m)
    
    % Compute position errors (desired - actual)
    pos_error_x = desired_pos(1) - ukf_pos_x;
    pos_error_y = desired_pos(2) - ukf_pos_y;
    pos_error_z = desired_pos(3) - ukf_pos_z;
    
    % P controller
    vel_cmd_x = controller_config.gains_xy * pos_error_x;
    vel_cmd_y = controller_config.gains_xy * pos_error_y;
    vel_cmd_z = controller_config.gains_z * pos_error_z;
    
    % Saturate velocity commands
    vel_cmd_x = max(-controller_config.vel_max_xy, min(controller_config.vel_max_xy, vel_cmd_x));
    vel_cmd_y = max(-controller_config.vel_max_xy, min(controller_config.vel_max_xy, vel_cmd_y));
    vel_cmd_z = max(-controller_config.vel_max_z, min(controller_config.vel_max_z, vel_cmd_z));
    
end