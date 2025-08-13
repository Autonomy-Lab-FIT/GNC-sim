function px4_arm_drone(client, config)
%PX4_ARM_DRONE Arm the drone motors
%   Sends command to arm the drone for flight

    px4_send_vehicle_command(client, config.MAV_CMD_COMPONENT_ARM_DISARM, ...
                            1.0, 0.0, config);
end