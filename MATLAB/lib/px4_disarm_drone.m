function px4_disarm_drone(client, config)
%PX4_DISARM_DRONE Disarm the drone motors
%   Sends command to disarm the drone

    px4_send_vehicle_command(client, config.MAV_CMD_COMPONENT_ARM_DISARM, ...
                            0.0, 0.0, config);
end