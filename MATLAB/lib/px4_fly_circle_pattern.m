function px4_fly_circle_pattern(client, config, status_callback)
%PX4_FLY_CIRCLE_PATTERN Execute a circular flight pattern

    % Generate circle points
    angles = linspace(0, 2*pi, config.circle_points+1);
    angles = angles(1:end-1);
    
    % Convert to cartesian coordinates
    x = config.circle_radius * cos(angles);
    y = config.circle_radius * sin(angles);
    z = repmat(config.takeoff_altitude, size(x));
    
    % Fly to each point on the circle
    for i = 1:length(x)
        yaw = atan2(-y(i), -x(i));
        
        status_callback(sprintf('Circle point %d/%d: [%.1f, %.1f, %.1f]', ...
                               i, length(x), x(i), y(i), z(i)));
        
        px4_send_trajectory(client, x(i), y(i), z(i), yaw, config);
        pause(config.circle_pause);
    end
    
    % Return to center
    px4_send_trajectory(client, 0, 0, config.takeoff_altitude, 0, config);
end