function [pos_ref, vel_ref] = get_mpc_reference(waypoints, current_time, dt)
    % Get current target
    target = interpolate_trajectory(waypoints, current_time);
    pos_ref = target(1:3);
    
    % Compute reference velocity from trajectory derivatives
    if current_time + dt <= waypoints(end, 5)
        target_next = interpolate_trajectory(waypoints, current_time + dt);
        vel_ref = (target_next(1:3) - target(1:3)) / dt;
    else
        vel_ref = [0; 0; 0]; % Stop at end
    end
end