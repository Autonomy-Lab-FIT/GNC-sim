function plot_cartesian(x_vals, y_vals, options)
%PLOT_CARTESIAN Create Cartesian scatter plot of obstacles
    
    % Plot obstacles
    scatter(x_vals, y_vals, 20, 'r', 'filled');
    hold on;
    
    % Show drone position and orientation
    if options.show_drone
        % Drone at origin
        plot(0, 0, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
        
        % Drone orientation arrow (pointing forward)
        arrow_length = options.max_range * 0.1;
        quiver(0, 0, arrow_length, 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.3);
    end
    
    % Set equal aspect ratio and limits
    axis equal;
    xlim([-options.max_range, options.max_range]);
    ylim([-options.max_range, options.max_range]);
    
    % Labels and formatting
    xlabel('Forward Distance (m)');
    ylabel('Lateral Distance (m)');
    title([options.title_text ' - Body Frame View']);
    
    if options.grid_on
        grid on;
    end
    
    % Add reference lines
    line([0, 0], ylim, 'Color', [0.5, 0.5, 0.5], 'LineStyle', '--');
    line(xlim, [0, 0], 'Color', [0.5, 0.5, 0.5], 'LineStyle', '--');
    
    hold off;
end