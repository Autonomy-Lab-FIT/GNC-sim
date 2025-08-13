function telemetry = px4_get_telemetry(client, config)
%PX4_GET_TELEMETRY Request and receive telemetry data from drone

    telemetry = [];

    try
        % Clear any pending data
        while client.NumBytesAvailable >= 4
            header = read(client, 4);
            msg_len = swapbytes(typecast(uint8(header), 'uint32'));
            if msg_len > 0 && msg_len < config.max_response_size && client.NumBytesAvailable >= msg_len
                read(client, msg_len);
            else
                break;
            end
        end

        % Send telemetry request
        command = struct('type', 'get_telemetry');
        json_data = jsonencode(command);
        msg_len = length(json_data);
        header = typecast(swapbytes(uint32(msg_len)), 'uint8');

        write(client, header);
        write(client, uint8(json_data));

        % Read responses until we get telemetry
        client.Timeout = config.timeout_telemetry;
        
        for attempt = 1:5
            try
                response_header = read(client, 4);
                if isempty(response_header)
                    continue;
                end

                response_len = swapbytes(typecast(uint8(response_header), 'uint32'));
                if response_len > config.max_response_size
                    continue;
                end

                response_data = read(client, response_len);
                if isempty(response_data)
                    continue;
                end

                response = jsondecode(char(response_data));

                if strcmp(response.type, 'telemetry')
                    telemetry = response;
                    break;
                elseif strcmp(response.type, 'error')
                    warning('px4_get_telemetry:bridge_error', 'Bridge error: %s', response.message);
                    return;
                end

            catch
                continue;
            end
        end

        if isempty(telemetry)
            warning('px4_get_telemetry:no_data', 'No telemetry data received');
        end

    catch e
        warning('px4_get_telemetry:error', 'Error getting telemetry: %s', e.message);
        telemetry = [];
    end
end