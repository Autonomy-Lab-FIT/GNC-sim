function fig_handle = visualize_lidar_scan(lidar_data, options)
%VISUALIZE_LIDAR_SCAN Visualize LiDAR scan data
%   Creates standard robotics visualization of LiDAR obstacles
%
%   Inputs:
%       lidar_data - Complete LiDAR data from px4_get_lidar_scan
%       options    - Optional parameters
%
%   Output:
%       fig_handle - Figure handle for the visualization

    arguments
        lidar_data
        options.plot_type = 'cartesian'     % 'cartesian', 'polar', or 'both'
        options.max_range = 15.0            % Maximum range to display
        options.show_drone = true           % Show drone position/orientation
        options.grid_on = true              % Show grid
        options.title_text = 'LiDAR Scan'  % Plot title
    end

    fig_handle = [];

    try
        % Validate input
        if isempty(lidar_data) || ~isfield(lidar_data, 'processed_obstacles')
            warning('visualize_lidar_scan:no_data', 'No LiDAR data provided');
            return;
        end

        obstacles = lidar_data.processed_obstacles;
        
        if isempty(obstacles)
            warning('visualize_lidar_scan:no_obstacles', 'No obstacles to visualize');
            return;
        end

        % Extract obstacle coordinates
        x_vals = [obstacles.x];
        y_vals = [obstacles.y];
        distances = [obstacles.distance];
        
        % Filter by max range
        valid_idx = distances <= options.max_range;
        x_vals = x_vals(valid_idx);
        y_vals = y_vals(valid_idx);
        distances = distances(valid_idx);

        % Create figure
        fig_handle = figure('Name', options.title_text, 'NumberTitle', 'off');

        if strcmp(options.plot_type, 'both')
            % Create subplots for both views
            subplot(1, 2, 1);
            plot_cartesian(x_vals, y_vals, options);
            
            subplot(1, 2, 2);
            plot_polar(obstacles, options);
        elseif strcmp(options.plot_type, 'polar')
            plot_polar(obstacles, options);
        else
            plot_cartesian(x_vals, y_vals, options);
        end

    catch e
        warning('visualize_lidar_scan:error', 'Error visualizing LiDAR scan: %s', e.message);
        fig_handle = [];
    end
end