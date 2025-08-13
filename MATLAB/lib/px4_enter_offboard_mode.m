function px4_enter_offboard_mode(client, config)
%PX4_ENTER_OFFBOARD_MODE Switch drone to offboard control mode
%   Sends command to enable offboard mode for external control

    px4_send_vehicle_command(client, config.MAV_CMD_DO_SET_MODE, ...
                            config.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, ...
                            config.PX4_CUSTOM_MAIN_MODE_OFFBOARD, config);
end