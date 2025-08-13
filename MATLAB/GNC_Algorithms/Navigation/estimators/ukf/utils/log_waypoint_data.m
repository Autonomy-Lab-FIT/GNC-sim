function data = log_waypoint_data(data, step, t, x_ukf, telemetry, current_target, config)
    % Extended data logging for waypoint analysis
    data = log_data(data, step, t, x_ukf, telemetry, config); % Use existing function
    
    % Add trajectory-specific data
    if step == 1
        data.commanded_x = zeros(size(data.time));
        data.commanded_y = zeros(size(data.time));
        data.commanded_z = zeros(size(data.time));
        data.commanded_yaw = zeros(size(data.time));
    end
    
    data.commanded_x(step) = current_target(1);
    data.commanded_y(step) = current_target(2);
    data.commanded_z(step) = current_target(3);
    data.commanded_yaw(step) = current_target(4);
end