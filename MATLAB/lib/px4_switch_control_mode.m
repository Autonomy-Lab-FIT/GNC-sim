function px4_switch_control_mode(client, mode, config)
    %PX4_SWITCH_CONTROL_MODE Switch between position and rate control
    %   mode: 'position', 'rates' and 'attitude'
    
    valid_modes = {'position', 'rates', 'attitude'};
    if ~ismember(mode, valid_modes)
        error('Invalid control mode. Use: position, rates, or attitude');
    end

    command = struct('type', 'switch_control_mode', ...
                     'mode', mode); 
    
    px4_send_json_command(client, command, config);
    
    fprintf('Switched to %s control mode\n', mode);
end