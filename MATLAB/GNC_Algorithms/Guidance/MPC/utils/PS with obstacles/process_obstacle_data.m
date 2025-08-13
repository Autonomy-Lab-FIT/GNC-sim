function obstacle_positions = process_obstacle_data(obstacle_data, mpc_config)
%PROCESS_OBSTACLE_DATA Convert LiDAR data to N closest obstacles from M sectors
    
    % Initialize with zeros for the N closest obstacles
    obstacle_positions = zeros(mpc_config.num_obstacles * 2, 1);
    
    if isempty(obstacle_data) || ~isfield(obstacle_data, 'processed_obstacles')
        return;
    end
    
    obstacles = obstacle_data.processed_obstacles;
    if isempty(obstacles)
        return;
    end
    
    % 1. Filter obstacles within 15m detection range
    valid_obstacles = [];
    for i = 1:length(obstacles)
        if obstacles(i).distance <= 15.0
            valid_obstacles = [valid_obstacles; obstacles(i)];
        end
    end
    
    if isempty(valid_obstacles)
        return;
    end

    % 2. Select one closest obstacle per sector from all valid obstacles
    sector_obstacles = [];
    sector_angle_rad = 2*pi / mpc_config.num_sectors;
    
    for sector = 1:mpc_config.num_sectors
        sector_start = (sector - 1) * sector_angle_rad - pi;
        sector_end = sector * sector_angle_rad - pi;
        
        current_sector_obstacles = [];
        for i = 1:length(valid_obstacles)
            angle = valid_obstacles(i).angle;
            if angle >= sector_start && angle < sector_end
                current_sector_obstacles = [current_sector_obstacles; valid_obstacles(i)];
            end
        end
        
        if ~isempty(current_sector_obstacles)
            [~, min_idx] = min([current_sector_obstacles.distance]);
            sector_obstacles = [sector_obstacles; current_sector_obstacles(min_idx)];
        end
    end

    if isempty(sector_obstacles)
        return;
    end

    % 3. Sort the sectored obstacles by distance
    [~, sorted_indices] = sort([sector_obstacles.distance]);
    
    % 4. Select the N closest obstacles for the solver
    num_to_select = min(length(sorted_indices), mpc_config.num_obstacles);
    
    for i = 1:num_to_select
        selected_obstacle = sector_obstacles(sorted_indices(i));
        idx = (i - 1) * 2;
        obstacle_positions(idx + 1) = selected_obstacle.x;
        obstacle_positions(idx + 2) = selected_obstacle.y;
    end
    % Any remaining positions in the vector will be (0,0)
end