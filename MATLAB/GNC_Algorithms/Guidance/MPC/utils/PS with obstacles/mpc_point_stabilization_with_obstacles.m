function [pos_cmd, vel_cmd, mpc_state] = mpc_point_stabilization_with_obstacles(target_position, current_state, mpc_config, mpc_state, obstacle_data)
%MPC_POINT_STABILIZATION_WITH_OBSTACLES Model Predictive Controller with obstacle avoidance
    import casadi.*
    
    % Initialize solver if it's the first run or config changed
    if isempty(mpc_state) || ~isfield(mpc_state, 'solver')
        fprintf('Initializing MPC obstacle avoidance solver...\n');
        mpc_state = initialize_mpc_solver_with_obstacles(mpc_config);
        fprintf('MPC obstacle avoidance solver initialized successfully\n');
    end
    
    x0 = current_state(1:6);
    
    if length(target_position) ~= 3
        error('Target position must be 3D [x, y, z]');
    end
    xs = [target_position(:); 0; 0; 0];
    
    % Process LiDAR data into obstacle positions
    obstacle_positions = process_obstacle_data(obstacle_data, mpc_config);
    
    % Update constraint bounds based on active obstacles
    args = mpc_state.args;
    args = update_obstacle_bounds(args, obstacle_positions, mpc_config);
    
    % Set solver parameters [x0; xs; obstacle_positions]
    args.p = [x0; xs; obstacle_positions];
    
    if isfield(mpc_state, 'prev_solution')
        args.x0 = mpc_state.prev_solution;
    end
    
    try
        tic;
        sol = mpc_state.solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx, ...
                              'lbg', args.lbg, 'ubg', args.ubg, 'p', args.p);
        solve_time = toc;
        
        if ~isempty(sol) && isfield(sol, 'x')
            X_opt = reshape(full(sol.x(1:mpc_config.n_states*(mpc_config.N+1))), ...
                           mpc_config.n_states, mpc_config.N+1);
            U_opt = reshape(full(sol.x(mpc_config.n_states*(mpc_config.N+1)+1:end)), ...
                           mpc_config.n_controls, mpc_config.N);
            
            guidance_horizon_steps = round(mpc_config.guidance_horizon_time / mpc_config.dt);
            guidance_step_index = min(guidance_horizon_steps + 1, size(X_opt, 2));
            
            pos_cmd = X_opt(1:3, guidance_step_index);  
            vel_cmd = X_opt(4:6, guidance_step_index);
            
            X_next = [X_opt(:, 2:end), X_opt(:, end)];
            U_next = [U_opt(:, 2:end), U_opt(:, end)];
            mpc_state.prev_solution = [reshape(X_next, [], 1); reshape(U_next, [], 1)];
            mpc_state.solve_time = solve_time;
        else
            error('Solver returned empty solution');
        end
        
    catch ME
        warning('MPC solver failed: %s', ME.message);
        pos_cmd = target_position(:);
        vel_cmd = [0; 0; 0];
        mpc_state.solve_time = NaN;
    end
end