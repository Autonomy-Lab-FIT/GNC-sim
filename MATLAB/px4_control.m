function px4_control(ip_address)
%PX4_CONTROL_SIMPLE Simple interface to control PX4 drone via ROS2 bridge
%   This function provides a basic interface to control a PX4 drone via the
%   ROS2 bridge. It establishes a connection to the bridge and allows the
%   user to send commands to the drone.
%
%   Usage:
%      px4_control_simple('192.168.1.147') % Replace with container IP

    % Add lib folder to path
    current_dir = fileparts(mfilename('fullpath'));
    lib_path = fullfile(current_dir, 'lib');
    if exist(lib_path, 'dir')
        addpath(lib_path);
    else
        error('Library folder not found. Please ensure lib/ folder exists with required functions.');
    end

    % Check input arguments
    if nargin < 1
        error('Please provide the IP address of the PX4-ROS2 bridge server');
    end

    % Configuration
    config = px4_get_config();
    
    % Create TCP client
    client = px4_connect(ip_address, config.port);
    
    % Create GUI
    [fig, ~] = px4_create_gui(client, config);
    
    % Wait for figure to close
    uiwait(fig);

    % Clean up
    px4_cleanup(client);
    
    % Remove lib path when done (optional cleanup)
    rmpath(lib_path);
end
