function error_distance = px4_get_position_error(client, config, target_x, target_y)
%PX4_GET_POSITION_ERROR Calculate current position error from target
%   Returns horizontal distance error in meters
    
    telemetry = px4_get_telemetry(client, config);
    
    if ~isempty(telemetry) && isfield(telemetry, 'local_position')
        pos = telemetry.local_position;
        error_distance = sqrt((pos.x - target_x)^2 + (pos.y - target_y)^2);
    else
        error_distance = inf; % No position data
    end
end