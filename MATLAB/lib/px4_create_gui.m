function [fig, status_text] = px4_create_gui(client, config)
%PX4_CREATE_GUI Create the main control GUI
%   Creates the figure window and all control buttons for drone operation

    % Create main figure
    fig = figure('Name', 'PX4 Control', 'NumberTitle', 'off', ...
                'Position', config.figure_position);

    % Create button panel
    button_panel = uipanel('Position', config.panel_position);

    % Create status text
    status_text = uicontrol(button_panel, 'Style', 'text', ...
                           'String', 'Status: Connected', ...
                           'Position', [20, 350, 350, 20], ...
                           'HorizontalAlignment', 'left');

    % Store data in figure
    setappdata(fig, 'client', client);
    setappdata(fig, 'config', config);
    setappdata(fig, 'status_text', status_text);

    % Create control buttons
    create_buttons(button_panel, fig);
    
    % Set figure close callback
    set(fig, 'CloseRequestFcn', @(~,~) close_figure(fig));
end

function create_buttons(parent, fig)
    % Mode and Arming buttons
    uicontrol(parent, 'Style', 'pushbutton', 'String', 'Enter Offboard Mode', ...
             'Position', [20, 300, 120, 30], ...
             'Callback', @(~,~) button_callback(fig, @px4_enter_offboard_mode, 'Offboard mode'));

    uicontrol(parent, 'Style', 'pushbutton', 'String', 'Arm Drone', ...
             'Position', [150, 300, 80, 30], ...
             'Callback', @(~,~) button_callback(fig, @px4_arm_drone, 'Arm'));

    uicontrol(parent, 'Style', 'pushbutton', 'String', 'Disarm Drone', ...
             'Position', [240, 300, 90, 30], ...
             'Callback', @(~,~) button_callback(fig, @px4_disarm_drone, 'Disarm'));

    % Flight control buttons
    uicontrol(parent, 'Style', 'pushbutton', 'String', 'Take Off (2m)', ...
             'Position', [50, 250, 120, 30], ...
             'Callback', @(~,~) button_callback(fig, @px4_takeoff, 'Takeoff'));

    uicontrol(parent, 'Style', 'pushbutton', 'String', 'Land', ...
             'Position', [200, 250, 120, 30], ...
             'Callback', @(~,~) button_callback(fig, @px4_land, 'Land'));

    % Pattern buttons
    uicontrol(parent, 'Style', 'pushbutton', 'String', 'Square Pattern', ...
             'Position', [50, 200, 120, 30], ...
             'Callback', @(~,~) pattern_callback(fig, @px4_fly_square_pattern, 'Square pattern'));

    uicontrol(parent, 'Style', 'pushbutton', 'String', 'Circle Pattern', ...
             'Position', [200, 200, 120, 30], ...
             'Callback', @(~,~) pattern_callback(fig, @px4_fly_circle_pattern, 'Circle pattern'));

    % Telemetry and control buttons
    uicontrol(parent, 'Style', 'pushbutton', 'String', 'Get Telemetry', ...
             'Position', [125, 150, 120, 30], ...
             'Callback', @(~,~) telemetry_callback(fig));

    uicontrol(parent, 'Style', 'pushbutton', 'String', 'Close Connection', ...
             'Position', [125, 100, 120, 30], ...
             'Callback', @(~,~) close_connection_callback(fig));
end

function button_callback(fig, func, action_name)
    client = getappdata(fig, 'client');
    config = getappdata(fig, 'config');
    status_text = getappdata(fig, 'status_text');
    
    try
        set(status_text, 'String', ['Status: ' action_name ' in progress']);
        func(client, config);
        set(status_text, 'String', ['Status: ' action_name ' completed']);
    catch e
        set(status_text, 'String', ['Status: Error - ' e.message]);
    end
end

function pattern_callback(fig, func, pattern_name)
    client = getappdata(fig, 'client');
    config = getappdata(fig, 'config');
    status_text = getappdata(fig, 'status_text');
    
    update_status = @(msg) set(status_text, 'String', ['Status: ' msg]);
    
    try
        set(status_text, 'String', ['Status: Starting ' pattern_name]);
        func(client, config, update_status);
        set(status_text, 'String', ['Status: ' pattern_name ' completed']);
    catch e
        set(status_text, 'String', ['Status: Error - ' e.message]);
    end
end

function telemetry_callback(fig)
    client = getappdata(fig, 'client');
    config = getappdata(fig, 'config');
    status_text = getappdata(fig, 'status_text');
    
    try
        set(status_text, 'String', 'Status: Requesting telemetry');
        telemetry = px4_get_telemetry(client, config);
    
        if ~isempty(telemetry) && isfield(telemetry, 'local_position') && ~isempty(telemetry.local_position)
            pos = telemetry.local_position;
            fprintf('\n--- Telemetry Data ---\n');
            fprintf('Position: [%.2f, %.2f, %.2f] m\n', pos.x, pos.y, pos.z);
            fprintf('Velocity: [%.2f, %.2f, %.2f] m/s\n', pos.vx, pos.vy, pos.vz);
            fprintf('Heading: %.1f degrees\n', rad2deg(pos.heading));
            fprintf('--------------------\n\n');
            set(status_text, 'String', sprintf('Status: Position [%.2f, %.2f, %.2f] m', pos.x, pos.y, pos.z));
        else
            fprintf('No telemetry data available\n');
            set(status_text, 'String', 'Status: No telemetry data available');
        end
    catch e
        set(status_text, 'String', ['Status: Error - ' e.message]);
    end
end

function close_connection_callback(fig)
    client = getappdata(fig, 'client');
    status_text = getappdata(fig, 'status_text');
    
    try
        px4_cleanup(client);
        set(status_text, 'String', 'Status: Connection closed');
    catch e
        set(status_text, 'String', ['Status: Error - ' e.message]);
    end
end

function close_figure(fig)
    try
        client = getappdata(fig, 'client');
        px4_cleanup(client);
    catch
    end
    delete(fig);
end