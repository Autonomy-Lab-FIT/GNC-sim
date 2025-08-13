function px4_takeoff(client, config)
%PX4_TAKEOFF Command drone to take off to default altitude
%   Sends trajectory setpoint for takeoff

    px4_send_trajectory(client, 0, 0, config.takeoff_altitude, 0, config);
end