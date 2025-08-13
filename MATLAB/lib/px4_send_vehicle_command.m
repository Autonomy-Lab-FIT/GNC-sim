function px4_send_vehicle_command(client, command_id, param1, param2, config)
%PX4_SEND_VEHICLE_COMMAND Send MAVLink vehicle command
%   Sends generic vehicle commands like ARM/DISARM, SET_MODE, etc.
%
%   Inputs:
%       client     - TCP client object
%       command_id - MAVLink command ID
%       param1     - First parameter
%       param2     - Second parameter
%       config     - Configuration structure

    % Create command message
    command = struct('type', 'send_command', ...
                     'command', command_id, ...
                     'param1', param1, ...
                     'param2', param2);

    % Send command and get response
    px4_send_json_command(client, command, config);
end