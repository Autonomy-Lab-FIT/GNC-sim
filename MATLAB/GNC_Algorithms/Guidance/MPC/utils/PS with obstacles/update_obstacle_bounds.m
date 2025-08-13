function args = update_obstacle_bounds(args, obstacle_positions, mpc_config)
%UPDATE_OBSTACLE_BOUNDS Set constraint bounds based on active/inactive obstacles
    safe_distance_sq = 9.0; % 3 meters squared
    n_obs_constraints = mpc_config.num_obstacles * (mpc_config.N + 1);
    
    n_original_constraints = length(args.lbg) - n_obs_constraints;
    
    for obs = 1:mpc_config.num_obstacles
        obs_x = obstacle_positions((obs-1)*2 + 1);
        obs_y = obstacle_positions((obs-1)*2 + 2);
        
        if obs_x == 0 && obs_y == 0
            constraint_bound = 0.0; % Inactive
        else
            constraint_bound = safe_distance_sq; % Active
        end
        
        for k = 1:mpc_config.N+1
            constraint_idx = n_original_constraints + (k-1)*mpc_config.num_obstacles + obs;
            args.lbg(constraint_idx) = constraint_bound;
        end
    end
end