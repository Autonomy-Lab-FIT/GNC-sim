function px4_send_acceleration_setpoint(client, x, y, z, vx, vy, vz, ax, ay, az, yaw, config)
%PX4_SEND_ACCELERATION_SETPOINT Send trajectory setpoint with acceleration commands
%   Sends position, velocity, acceleration, and yaw setpoint commands to the drone
%
%   Inputs:
%       client - TCP client object
%       x, y, z - Position coordinates (NED frame, Z negative up)
%       vx, vy, vz - Velocity commands (NED frame, m/s)
%       ax, ay, az - Acceleration commands (NED frame, m/s²)
%       yaw - Heading angle in radians
%       config - Configuration structure

    % Create command message with acceleration setpoints
    command = struct('type', 'set_trajectory', ...
                    'data', struct('x', x, 'y', y, 'z', z, ...
                                  'vx', vx, 'vy', vy, 'vz', vz, ...
                                  'ax', ax, 'ay', ay, 'az', az, ...
                                  'yaw', yaw));
    
    % Send command and get response
    px4_send_json_command(client, command, config);
end