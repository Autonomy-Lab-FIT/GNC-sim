function lidar_data = px4_get_lidar_scan(client, config, options)
%PX4_GET_LIDAR_SCAN Request and receive LiDAR scan data from drone
%   Gets LiDAR scan with configurable processing parameters
%
%   Inputs:
%       client  - TCP client object
%       config  - Configuration structure
%       options - Optional parameters (name-value pairs)
%
%   Output:
%       lidar_data - Structure containing LiDAR scan data

    arguments
        client
        config
        options.filter_invalid = config.lidar_filter_invalid
        options.min_range = config.lidar_min_range
        options.max_range = config.lidar_max_range
        options.subsample_rate = config.lidar_subsample_rate
        options.include_raw = config.lidar_include_raw
        options.include_processed = config.lidar_include_processed
        options.invalid_range_value = config.lidar_invalid_range_value
    end

    lidar_data = [];

    try
        % Parameter validation
        if options.min_range <= 0
            warning('px4_get_lidar_scan:invalid_param', 'min_range must be positive, using default');
            options.min_range = config.lidar_min_range;
        end
        
        if options.max_range <= options.min_range
            warning('px4_get_lidar_scan:invalid_param', 'max_range must be greater than min_range, using defaults');
            options.max_range = config.lidar_max_range;
            options.min_range = config.lidar_min_range;
        end
        
        if options.subsample_rate < 1 || mod(options.subsample_rate, 1) ~= 0
            warning('px4_get_lidar_scan:invalid_param', 'subsample_rate must be positive integer, using default');
            options.subsample_rate = config.lidar_subsample_rate;
        end

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

        % Send LiDAR scan request
        command = struct('type', 'get_lidar_scan', ...
                        'filter_invalid', options.filter_invalid, ...
                        'min_range', options.min_range, ...
                        'max_range', options.max_range, ...
                        'subsample_rate', options.subsample_rate, ...
                        'include_raw', options.include_raw, ...
                        'include_processed', options.include_processed, ...
                        'invalid_range_value', options.invalid_range_value);

        json_data = jsonencode(command);
        msg_len = length(json_data);
        header = typecast(swapbytes(uint32(msg_len)), 'uint8');

        write(client, header);
        write(client, uint8(json_data));

        % Read responses until we get LiDAR data
        client.Timeout = config.lidar_timeout;
        
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

                if strcmp(response.type, 'lidar_scan')
                    lidar_data = response;
                    break;
                elseif strcmp(response.type, 'error')
                    warning('px4_get_lidar_scan:bridge_error', 'Bridge error: %s', response.message);
                    return;
                end

            catch
                continue;
            end
        end

        if isempty(lidar_data)
            warning('px4_get_lidar_scan:no_data', 'No LiDAR data received');
        end

    catch e
        warning('px4_get_lidar_scan:error', 'Error getting LiDAR scan: %s', e.message);
        lidar_data = [];
    end
end