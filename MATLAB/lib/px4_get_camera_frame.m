function frame = px4_get_camera_frame(client, config)
%PX4_GET_CAMERA_FRAME Request camera frame using config settings
%   Uses compression settings from config, no per-request parameters

    % Create command using config settings
    command = struct('type', 'get_camera_frame', ...
                     'compress', config.camera_compress, ...
                     'timeout_ms', config.camera_timeout_ms);
    
    if config.camera_compress
        command.jpeg_quality = config.camera_jpeg_quality;
    end

    try
        frame = px4_send_json_command(client, command, config);
    catch e
        error('Camera frame request failed: %s', e.message);
    end
end