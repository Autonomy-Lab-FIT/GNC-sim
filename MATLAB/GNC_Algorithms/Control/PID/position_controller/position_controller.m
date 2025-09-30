function [vel_cmd_x, vel_cmd_y, vel_cmd_z] = position_controller(desired_pos, ukf_state, controller_config)

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