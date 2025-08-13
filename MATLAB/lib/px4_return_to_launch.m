function success = px4_return_to_launch(client, config)
%PX4_RETURN_TO_LAUNCH Start automatic return to launch sequence
% Commands drone to return to takeoff location and land
% https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_RETURN_TO_LAUNCH

    try
        px4_send_vehicle_command(client, 20, 0, 0, config); % MAV_CMD_NAV_RETURN_TO_LAUNCH
        success = true;
    catch
        success = false;
    end
end