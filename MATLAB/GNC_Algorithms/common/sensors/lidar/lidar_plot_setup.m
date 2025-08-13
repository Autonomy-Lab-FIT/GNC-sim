function plot_handles = lidar_plot_setup(options)
%LIDAR_PLOT_SETUP Initialize real-time LiDAR plot
%   Sets up figure and plot elements for real-time LiDAR visualization
%
%   Inputs:
%       options - Optional parameters
%
%   Output:
%       plot_handles - Structure containing all plot handles

    arguments
        options.figure_name = 'Real-time LiDAR'
        options.max_range = 15.0
        options.show_drone = true
        options.show_grid = true
    end

    try
        % Create figure
        fig = figure('Name', options.figure_name, 'NumberTitle', 'off');
        ax = axes('Parent', fig);
        hold(ax, 'on');
        
        % Initialize plot elements
        obstacle_plot = scatter(ax, [], [], 20, 'r', 'filled');
        
        % Drone visualization
        if options.show_drone
            drone_plot = plot(ax, 0, 0, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
            arrow_plot = quiver(ax, 0, 0, options.max_range*0.1, 0, 'b', ...
                               'LineWidth', 2, 'MaxHeadSize', 0.3);
        else
            drone_plot = [];
            arrow_plot = [];
        end
        
        % Setup axes
        axis(ax, 'equal');
        xlim(ax, [-options.max_range, options.max_range]);
        ylim(ax, [-options.max_range, options.max_range]);
        xlabel(ax, 'Forward Distance (m)');
        ylabel(ax, 'Lateral Distance (m)');
        title(ax, 'LiDAR Scan - Body Frame View');
        
        if options.show_grid
            grid(ax, 'on');
            % Reference lines
            line(ax, [0, 0], ylim(ax), 'Color', [0.5, 0.5, 0.5], 'LineStyle', '--');
            line(ax, xlim(ax), [0, 0], 'Color', [0.5, 0.5, 0.5], 'LineStyle', '--');
        end
        
        % Status text
        status_text = text(ax, -options.max_range*0.9, options.max_range*0.9, ...
                          'Initializing...', 'FontSize', 10, ...
                          'BackgroundColor', 'w', 'EdgeColor', 'k');

        % Build return structure
        plot_handles = struct();
        plot_handles.figure = fig;
        plot_handles.axes = ax;
        plot_handles.obstacle_plot = obstacle_plot;
        plot_handles.drone_plot = drone_plot;
        plot_handles.arrow_plot = arrow_plot;
        plot_handles.status_text = status_text;
        plot_handles.max_range = options.max_range;

    catch e
        warning('lidar_plot_setup:error', 'Error setting up plot: %s', e.message);
        plot_handles = [];
    end
end