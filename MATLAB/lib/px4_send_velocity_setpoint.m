function px4_send_velocity_setpoint(client, x, y, z, vx, vy, vz, yaw, config)
%PX4_SEND_VELOCITY_SETPOINT Send trajectory setpoint with velocity commands
%   Sends position, velocity, and yaw setpoint commands to the drone
%
%   Inputs:
%       client - TCP client object
%       x, y, z - Position coordinates (NED frame, Z negative up)
%       vx, vy, vz - Velocity commands (NED frame, m/s)
%       yaw - Heading angle in radians
%       config - Configuration structure

    % Create command message with velocity setpoints
    command = struct('type', 'set_trajectory', ...
                    'data', struct('x', x, 'y', y, 'z', z, ...
                                  'vx', vx, 'vy', vy, 'vz', vz, ...
                                  'yaw', yaw));
    
    % Send command and get response
    px4_send_json_command(client, command, config);
end