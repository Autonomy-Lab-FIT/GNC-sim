function target = interpolate_trajectory(waypoints, current_time)
    % Find current position along trajectory
    times = waypoints(:, 5);
    
    if current_time <= times(1)
        target = waypoints(1, 1:4);
    elseif current_time >= times(end)
        target = waypoints(end, 1:4);
    else
        % Linear interpolation between waypoints
        idx = find(times >= current_time, 1, 'first');
        if idx == 1
            target = waypoints(1, 1:4);
        else
            t1 = times(idx-1);
            t2 = times(idx);
            alpha = (current_time - t1) / (t2 - t1);
            
            target = (1-alpha) * waypoints(idx-1, 1:4) + alpha * waypoints(idx, 1:4);
        end
    end
end