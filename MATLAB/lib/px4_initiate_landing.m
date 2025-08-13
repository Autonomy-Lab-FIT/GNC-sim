function success = px4_initiate_landing(client, config)
%PX4_INITIATE_LANDING Start automatic landing sequence
%   Sends landing command, does not wait for completion
% https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND

    try
        px4_send_vehicle_command(client, 21, 0, 0, config); % MAV_CMD_NAV_LAND
        success = true;
    catch
        success = false;
    end
end