function px4_send_attitude_setpoint(client, thrust, q_w, q_x, q_y, q_z, config, yaw_rate)
%PX4_SEND_ATTITUDE Send attitude setpoint (quaternion + thrust) to drone
%   Sends desired attitude quaternion and thrust command to the drone
%
%   Inputs:
%       client  - TCP client object
%       thrust  - Thrust force in Newtons [N]
%       q_w     - Quaternion scalar component
%       q_x     - Quaternion x component
%       q_y     - Quaternion y component
%       q_z     - Quaternion z component
%       config  - Configuration structure
%       yaw_rate - (Optional) Yaw rate feedforward in rad/s

    % Handle optional yaw_rate parameter
    if nargin < 8
        yaw_rate = [];
    end

    % Create command message
    command = struct('type', 'set_attitude', ...
                     'thrust', thrust, ...
                     'q_w', q_w, ...
                     'q_x', q_x, ...
                     'q_y', q_y, ...
                     'q_z', q_z);
    
    % Add optional yaw rate if provided
    if ~isempty(yaw_rate)
        command.yaw_rate = yaw_rate;
    end

    % Send command and get response
    px4_send_json_command(client, command, config);
end