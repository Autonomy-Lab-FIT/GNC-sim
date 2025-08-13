function px4_fly_square_pattern(client, config, status_callback)

%PX4_FLY_SQUARE_PATTERN Execute a square flight pattern

    % Define square corners

    square = [0, 0, config.takeoff_altitude;
              config.square_size, 0, config.takeoff_altitude;
              config.square_size, config.square_size, config.takeoff_altitude;
              0, config.square_size, config.takeoff_altitude;
              0, 0, config.takeoff_altitude];

    % Fly to each corner
    for i = 1:size(square, 1)
        status_callback(sprintf('Moving to waypoint %d: [%.1f, %.1f, %.1f]', ...
                               i, square(i,1), square(i,2), square(i,3)));
        px4_send_trajectory(client, square(i,1), square(i,2), square(i,3), 0, config); % send trajectory
        pause(config.waypoint_pause);
    end
end