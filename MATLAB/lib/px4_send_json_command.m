function response = px4_send_json_command(client, command, config)
%PX4_SEND_JSON_COMMAND Low-level function to send JSON commands
%   Handles the JSON encoding, message framing, and response parsing
%
%   Inputs:
%       client  - TCP client object
%       command - Command structure to send
%       config  - Configuration structure
%
%   Output:
%       response - Response structure (added)

    response = []; % Initialize response
    
    try
        % Convert to JSON
        json_data = jsonencode(command);
        msg_len = length(json_data);

        % Create header with message length in BIG-ENDIAN
        header = typecast(swapbytes(uint32(msg_len)), 'uint8');

        % Send header and data
        write(client, header);
        write(client, uint8(json_data));

        % Wait for response (now required for camera)
        try
            client.Timeout = config.timeout_response;
            response_header = read(client, 4);

            if ~isempty(response_header)
                response_len_bytes = typecast(uint8(response_header), 'uint32');
                response_len = swapbytes(response_len_bytes);

                % Sanity check length
                if response_len > config.max_response_size
                    disp(['Response too large: ' num2str(response_len) ' bytes']);
                    response = struct('type', 'error', 'message', 'Response too large');
                    return;
                end

                response_data = read(client, response_len);

                if ~isempty(response_data)
                    % Parse JSON response
                    response_json = char(response_data);
                    response = jsondecode(response_json);
                    
                    if config.debug_mode
                        disp(['Response: ' response_json]);
                    end
                else
                    response = struct('type', 'error', 'message', 'Empty response');
                end
            else
                response = struct('type', 'error', 'message', 'No response header');
            end
        catch e
            response = struct('type', 'error', 'message', ['Response read failed: ' e.message]);
        end
    catch e
        error(['Error sending command: ' e.message]);
    end
end