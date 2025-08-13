function [waypoints, duration] = generate_altitude_pattern(dt)
    % Altitude variation pattern
    duration = 60;
    t_vec = 0:dt:duration;
    
    % Fixed horizontal position, varying altitude
    x = zeros(size(t_vec));
    y = zeros(size(t_vec));
    
    % Altitude profile: ground -> hover -> climb -> descend -> hover
    z_profile = [
        0, -1, -2, -3, -4, -3, -2, -4, -5, -3, -2, -1, -2, -3, -2, -1, -2, -2, -2, -2, ...
        -2, -1, -2, -3, -4, -2, -1, -2, -3, -2, -1
    ];
    
    % Pad or trim to match time vector
    if length(z_profile) < length(t_vec)
        z = [z_profile, repmat(z_profile(end), 1, length(t_vec) - length(z_profile))];
    else
        z = z_profile(1:length(t_vec));
    end
    
    yaw = zeros(size(t_vec));
    
    waypoints = [x(:), y(:), z(:), yaw(:), t_vec(:)];
end