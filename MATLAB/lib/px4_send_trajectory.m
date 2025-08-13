function px4_send_trajectory(client, x, y, z, yaw, config)
%PX4_SEND_TRAJECTORY Send trajectory setpoint to drone
%   Sends position and yaw setpoint commands to the drone
%
%   Inputs:
%       client - TCP client object
%       x, y, z - Position coordinates (NED frame, Z negative up)
%       yaw     - Heading angle in radians
%       config  - Configuration structure

    % Create command message
    command = struct('type', 'set_trajectory', ...
                     'data', struct('x', x, 'y', y, 'z', z, 'yaw', yaw));

    % Send command and get response
    px4_send_json_command(client, command, config);
end