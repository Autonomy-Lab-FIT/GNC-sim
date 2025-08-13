function [landed, altitude] = px4_check_landing_status(client, config)
%PX4_CHECK_LANDING_STATUS Check if vehicle has landed
%   Returns: [landed_boolean, current_altitude]
    
    landed = false;
    altitude = 0;
    
    telemetry = px4_get_telemetry(client, config);
    
    if ~isempty(telemetry)
        if isfield(telemetry, 'status')
            landed = (telemetry.status.arming_state == 1); % 1 = disarmed
        end
        
        if isfield(telemetry, 'local_position')
            altitude = -telemetry.local_position.z;
        end
    end
end