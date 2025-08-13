function plot_polar(obstacles, options)
%PLOT_POLAR Create polar plot of LiDAR scan
    
    % Extract angles and distances
    angles = [obstacles.angle];
    distances = [obstacles.distance];
    
    % Filter by max range
    valid_idx = distances <= options.max_range;
    angles = angles(valid_idx);
    distances = distances(valid_idx);
    
    % Create polar plot
    polarscatter(angles, distances, 20, 'r', 'filled');
    
    % Set limits
    rlim([0, options.max_range]);
    
    % Labels
    title([options.title_text ' - Polar View']);
    
    % Add grid if requested
    if options.grid_on
        rticks(0:5:options.max_range);
        thetaticks(0:30:330);
    end
end