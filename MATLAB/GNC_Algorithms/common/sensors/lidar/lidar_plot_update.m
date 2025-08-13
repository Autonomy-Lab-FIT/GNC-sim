function success = lidar_plot_update(plot_handles, obstacles, options)
%LIDAR_PLOT_UPDATE Update real-time LiDAR plot with new data
%   Updates obstacle positions and status information
%
%   Inputs:
%       plot_handles - Plot handles from lidar_plot_setup
%       obstacles    - Obstacle array from px4_get_lidar_scan
%       options      - Optional parameters
%
%   Output:
%       success      - True if update successful

    arguments
        plot_handles
        obstacles
        options.status_text = ''
        options.force_update = false
    end

    success = false;

    try
        % Validate inputs
        if isempty(plot_handles) || ~ishandle(plot_handles.figure)
            warning('lidar_plot_update:invalid_handle', 'Invalid plot handles');
            return;
        end

        % Update obstacle positions
        if ~isempty(obstacles)
            x_vals = [obstacles.x];
            y_vals = [obstacles.y];
            
            % Filter by range
            distances = sqrt(x_vals.^2 + y_vals.^2);
            valid_idx = distances <= plot_handles.max_range;
            x_vals = x_vals(valid_idx);
            y_vals = y_vals(valid_idx);
            
            % Update plot
            set(plot_handles.obstacle_plot, 'XData', x_vals, 'YData', y_vals);
        else
            % Clear obstacles
            set(plot_handles.obstacle_plot, 'XData', [], 'YData', []);
        end

        % Update status text
        if ~isempty(options.status_text)
            set(plot_handles.status_text, 'String', options.status_text);
        end

        % Force graphics update
        if options.force_update
            drawnow;
        else
            drawnow limitrate;
        end

        success = true;

    catch e
        warning('lidar_plot_update:error', 'Error updating plot: %s', e.message);
    end
end