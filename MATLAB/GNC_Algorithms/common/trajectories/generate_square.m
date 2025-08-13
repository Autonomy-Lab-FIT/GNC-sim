function [waypoints, duration] = generate_square(altitude)
    % Square pattern with pauses at corners
    side_length = 6; % 6-meter sides
    corner_pause = 3; % 3 seconds at each corner
    transit_time = 8; % 8 seconds between corners
    
    corners = [
        0, 0, altitude, 0;
        side_length, 0, altitude, 0;
        side_length, side_length, altitude, 0;
        0, side_length, altitude, 0;
        0, 0, altitude, 0  % Return to start
    ];
    
    waypoints = [];
    t = 0;
    
    for i = 1:size(corners,1)-1
        % Add current corner
        waypoints = [waypoints; corners(i,:), t];
        t = t + corner_pause;
        
        % Add intermediate points for smooth transition
        n_points = 10;
        for j = 1:n_points
            alpha = j / n_points;
            interp_point = (1-alpha) * corners(i,:) + alpha * corners(i+1,:);
            waypoints = [waypoints; interp_point, t + alpha * transit_time];
        end
        t = t + transit_time;
    end
    
    % Final corner
    waypoints = [waypoints; corners(end,:), t];
    duration = t + corner_pause;
end