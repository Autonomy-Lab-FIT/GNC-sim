function grid_data = lidar_to_occupancy_grid(obstacles, options)
%LIDAR_TO_OCCUPANCY_GRID Convert LiDAR obstacles to occupancy grid
%   Converts processed obstacle data to occupancy grid representation
%
%   Inputs:
%       obstacles - Processed obstacles from px4_get_lidar_scan
%       options   - Optional parameters
%
%   Output:
%       grid_data - Structure containing occupancy grid and metadata

    arguments
        obstacles
        options.grid_size = 10.0        % Grid size in meters (10m x 10m)
        options.resolution = 0.1        % Grid resolution in meters (10cm cells)
        options.center_x = 0            % Grid center x coordinate
        options.center_y = 0            % Grid center y coordinate
        options.occupied_threshold = 0.5 % Threshold for occupied cell
    end

    grid_data = [];

    try
        % Validate inputs
        if isempty(obstacles)
            warning('lidar_to_occupancy_grid:no_data', 'No obstacle data provided');
            return;
        end

        % Calculate grid dimensions
        cells_per_side = round(options.grid_size / options.resolution);
        
        % Initialize grid (0 = free, 1 = occupied, -1 = unknown)
        grid = zeros(cells_per_side, cells_per_side);
        
        % Grid bounds
        half_size = options.grid_size / 2;
        min_x = options.center_x - half_size;
        max_x = options.center_x + half_size;
        min_y = options.center_y - half_size;
        max_y = options.center_y + half_size;

        % Process each obstacle
        occupied_cells = 0;
        
        for i = 1:length(obstacles)
            obs = obstacles(i);
            
            % Get obstacle position
            x = obs.x;
            y = obs.y;
            
            % Check if obstacle is within grid bounds
            if x >= min_x && x < max_x && y >= min_y && y < max_y
                % Convert to grid coordinates
                grid_x = floor((x - min_x) / options.resolution) + 1;
                grid_y = floor((y - min_y) / options.resolution) + 1;
                
                % Ensure within bounds
                if grid_x >= 1 && grid_x <= cells_per_side && ...
                   grid_y >= 1 && grid_y <= cells_per_side
                    grid(grid_y, grid_x) = 1;  % Mark as occupied
                    occupied_cells = occupied_cells + 1;
                end
            end
        end

        % Create coordinate arrays for plotting
        x_coords = linspace(min_x, max_x, cells_per_side + 1);
        y_coords = linspace(min_y, max_y, cells_per_side + 1);

        % Build result structure
        grid_data = struct();
        grid_data.grid = grid;
        grid_data.x_coords = x_coords;
        grid_data.y_coords = y_coords;
        grid_data.resolution = options.resolution;
        grid_data.grid_size = options.grid_size;
        grid_data.center = [options.center_x, options.center_y];
        grid_data.occupied_cells = occupied_cells;
        grid_data.total_cells = cells_per_side * cells_per_side;
        grid_data.occupancy_ratio = occupied_cells / (cells_per_side * cells_per_side);

    catch e
        warning('lidar_to_occupancy_grid:error', 'Error creating occupancy grid: %s', e.message);
        grid_data = [];
    end
end