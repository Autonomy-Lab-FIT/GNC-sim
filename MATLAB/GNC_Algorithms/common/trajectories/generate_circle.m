function [waypoints, duration] = generate_circle(altitude, dt)
    % Circular pattern
    duration = 50;
    t_vec = 0:dt:duration;
    
    % Circle parametric equations
    radius = 5; % 5-meter radius
    x = radius * cos(2*pi*t_vec/duration);
    y = radius * sin(2*pi*t_vec/duration);
    z = altitude * ones(size(t_vec));
    
    % Yaw points toward center
    yaw = atan2(-y, -x);
    
    waypoints = [x(:), y(:), z(:), yaw(:), t_vec(:)];
end