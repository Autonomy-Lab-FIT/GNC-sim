function [x, y, z] = px4_get_current_position(client, config)
%PX4_GET_CURRENT_POSITION Get current vehicle position
%   Returns NED coordinates [x, y, z]
    
    x = 0; y = 0; z = 0;
    
    telemetry = px4_get_telemetry(client, config);
    
    if ~isempty(telemetry) && isfield(telemetry, 'local_position')
        pos = telemetry.local_position;
        x = pos.x;
        y = pos.y; 
        z = pos.z;
    end
end