function [waypoints, duration] = generate_figure8(altitude, dt)
    % Figure-8 pattern: smooth continuous motion
    duration = 80; % 80 seconds for complete figure-8
    t_vec = 0:dt:duration;
    
    % Figure-8 parametric equations
    scale = 4; % 4-meter radius
    x = scale * sin(2*pi*t_vec/duration);
    y = scale * sin(4*pi*t_vec/duration); % Double frequency for figure-8
    z = altitude * ones(size(t_vec));
    
    % Yaw follows direction of motion
    dx = gradient(x);
    dy = gradient(y);
    yaw = atan2(dy, dx);
    
    waypoints = [x(:), y(:), z(:), yaw(:), t_vec(:)];
end