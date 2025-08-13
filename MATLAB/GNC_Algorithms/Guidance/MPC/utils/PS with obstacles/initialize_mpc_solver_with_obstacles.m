function mpc_state = initialize_mpc_solver_with_obstacles(mpc_config)
%INITIALIZE_MPC_SOLVER_WITH_OBSTACLES Sets up CasADi optimization problem
    import casadi.*
    
    U = SX.sym('U', mpc_config.n_controls, mpc_config.N);
    X = SX.sym('X', mpc_config.n_states, mpc_config.N+1);
    
    % Parameters: [x0; xs; obstacle_positions]
    % Total size = 12 (states) + num_obstacles * 2
    P = SX.sym('P', 2*mpc_config.n_states + mpc_config.num_obstacles * 2);
    
    obj = 0;
    g = [];
    
    g = [g; X(:,1) - P(1:mpc_config.n_states)];
    
    for k = 1:mpc_config.N
        st = X(:,k);
        con = U(:,k);
        
        state_error = st - P(mpc_config.n_states+1:2*mpc_config.n_states);
        obj = obj + state_error' * mpc_config.Q * state_error + con' * mpc_config.R * con;
        
        st_next = X(:,k+1);
        f_value = mpc_config.dynamics_function(st, con);
        st_next_euler = st + mpc_config.dt * f_value;
        g = [g; st_next - st_next_euler];
    end
    
    terminal_error = X(:,end) - P(mpc_config.n_states+1:2*mpc_config.n_states);
    obj = obj + terminal_error' * mpc_config.Qf * terminal_error;
    
    for k = 1:mpc_config.N+1
        pos_x = X(1, k);
        pos_y = X(2, k);
        
        for obs = 1:mpc_config.num_obstacles
            obs_x = P(2*mpc_config.n_states + (obs-1)*2 + 1);
            obs_y = P(2*mpc_config.n_states + (obs-1)*2 + 2);
            
            distance_sq = (pos_x - obs_x)^2 + (pos_y - obs_y)^2;
            g = [g; distance_sq];
        end
    end
    
    % Create optimization variables vector
    % EDITED: Replaced [] with explicit dimensions for CasADi compatibility
    OPT_variables = [reshape(X, mpc_config.n_states*(mpc_config.N+1), 1);
                     reshape(U, mpc_config.n_controls*mpc_config.N, 1)];
    
    % Create NLP problem
    nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);
    
    % Solver options
    opts = struct;
    opts.ipopt.max_iter = 200;
    opts.ipopt.print_level = 0;
    opts.print_time = 0;
    opts.ipopt.acceptable_tol = 1e-6;
    opts.ipopt.acceptable_obj_change_tol = 1e-4;
    opts.ipopt.warm_start_init_point = 'yes';
    
    % Create solver
    solver = nlpsol('solver', 'ipopt', nlp_prob, opts);
    
    % Set up constraint bounds (includes obstacle constraints)
    args = setup_constraints_with_obstacles(mpc_config, length(g), length(OPT_variables));
    
    % Store in persistent state
    mpc_state.solver = solver;
    mpc_state.args = args;
    mpc_state.prev_solution = args.x0;
end