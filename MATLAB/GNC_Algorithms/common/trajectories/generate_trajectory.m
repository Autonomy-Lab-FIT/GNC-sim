function [waypoints, duration] = generate_trajectory(config, trajectory_type, dt)
    
    altitude = config.takeoff_altitude; % Fixed altitude for most trajectories
    
    switch lower(trajectory_type)
        case 'figure8'
            [waypoints, duration] = generate_figure8(altitude, dt);
            
        case 'square'
            [waypoints, duration] = generate_square(altitude);
            
        case 'altitude'
            [waypoints, duration] = generate_altitude_pattern(dt);
            
        case 'circle'
            [waypoints, duration] = generate_circle(altitude, dt);
            
        otherwise
            error('Unknown trajectory type: %s. Use: figure8, square, altitude, circle', trajectory_type);
    end
end