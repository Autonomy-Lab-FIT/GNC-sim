function [pos_cmd, vel_cmd, mpc_state] = mpc_point_stabilization(target_position, current_state, mpc_config, mpc_state)
%MPC_POINT_STABILIZATION Model Predictive Controller for point stabilization
% Guidance - outputs position and velocity setpoints only

    import casadi.*
    
    % Initialize solver
    if isempty(mpc_state) || ~isfield(mpc_state, 'solver')
        fprintf('Initializing MPC point stabilization solver...\n');
        mpc_state = initialize_mpc_solver(mpc_config);
        fprintf('MPC solver initialized successfully\n');
    end
    
    % current state and reference
    x0 = current_state(1:6); % [px, py, pz, vx, vy, vz]
    
    % Point stabilization: target position with zero velocity
    if length(target_position) ~= 3
        error('Target position must be 3D [x, y, z]');
    end
    xs = [target_position(:); 0; 0; 0]; % [x, y, z, 0, 0, 0]
    
    % Set solver parameters
    args = mpc_state.args;
    args.p = [x0; xs];
    % fprintf('MPC Params - x0: [%.2f, %.2f, %.2f], xs: [%.2f, %.2f, %.2f]\n', ...
    %     x0(1), x0(2), x0(3), xs(1), xs(2), xs(3));
    
    % fprintf('P parameter check:\n');
    % fprintf('P(1:6) (x0): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', ...
    %     args.p(1), args.p(2), args.p(3), args.p(4), args.p(5), args.p(6));
    % fprintf('P(7:12) (xs): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', ...
    %     args.p(7), args.p(8), args.p(9), args.p(10), args.p(11), args.p(12));

    % Use warm start from previous solution
    if isfield(mpc_state, 'prev_solution')
        args.x0 = mpc_state.prev_solution;
    end
    
    try
        % Solve MPC optimization
        tic;
        sol = mpc_state.solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx, ...
                              'lbg', args.lbg, 'ubg', args.ubg, 'p', args.p);
        solve_time = toc;
        
        % Check if solution exists and extract data
        if ~isempty(sol) && isfield(sol, 'x')
            % Extract optimal trajectory and controls
            X_opt = reshape(full(sol.x(1:mpc_config.n_states*(mpc_config.N+1))), ...
                           mpc_config.n_states, mpc_config.N+1);
            U_opt = reshape(full(sol.x(mpc_config.n_states*(mpc_config.N+1)+1:end)), ...
                           mpc_config.n_controls, mpc_config.N);

            % fprintf('MPC Solution Check:\n');
            % fprintf('  X0 (current): [%.3f, %.3f, %.3f]\n', X_opt(1,1), X_opt(2,1), X_opt(3,1));
            % fprintf('  X1 (next):    [%.3f, %.3f, %.3f]\n', X_opt(1,2), X_opt(2,2), X_opt(3,2));
            % fprintf('  Target:       [%.3f, %.3f, %.3f]\n', target_position(1), target_position(2), target_position(3));
            % fprintf('  U0 (accel):   [%.3f, %.3f, %.3f]\n', U_opt(1,1), U_opt(2,1), U_opt(3,1));
            
            guidance_horizon_steps = round(mpc_config.guidance_horizon_time / mpc_config.dt);
            guidance_step_index = min(guidance_horizon_steps + 1, size(X_opt, 2));
            
            pos_cmd = X_opt(1:3, guidance_step_index);  
            vel_cmd = X_opt(4:6, guidance_step_index);
            
            % fprintf('Position Setpoint, z: %.2f\n', full(pos_cmd(3)));

            % Prepare warm start for next iteration
            X_next = [X_opt(:, 2:end), X_opt(:, end)];
            U_next = [U_opt(:, 2:end), U_opt(:, end)];
            mpc_state.prev_solution = [reshape(X_next, [], 1); reshape(U_next, [], 1)];
            
            % Store diagnostics
            mpc_state.solve_time = solve_time;
            
        else
            error('Solver returned empty solution');
        end
        
    catch ME
        warning('MPC solver failed: %s', ME.message);
        
        %  use target position as command
        pos_cmd = target_position(:);
        vel_cmd = [0; 0; 0];
        
        % Store failure info
        mpc_state.solve_time = NaN;
    end
end