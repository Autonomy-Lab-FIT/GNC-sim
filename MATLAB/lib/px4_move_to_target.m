function success = px4_move_to_target(client, config, x, y, altitude)
%PX4_MOVE_TO_TARGET Navigate to specific coordinates
%   Pure navigation function for guidance algorithms
    if nargin < 5, altitude = config.takeoff_altitude; end
    
    try
        px4_send_trajectory(client, x, y, altitude, 0, config);
        success = true;
    catch
        success = false;
    end
end
