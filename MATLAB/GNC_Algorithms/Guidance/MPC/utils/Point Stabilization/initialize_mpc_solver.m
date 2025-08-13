function mpc_state = initialize_mpc_solver(mpc_config)
%INITIALIZE_MPC_SOLVER Sets up CasADi optimization problem with Euler integration

    import casadi.*
    
    % Decision variables
    U = SX.sym('U', mpc_config.n_controls, mpc_config.N);
    X = SX.sym('X', mpc_config.n_states, mpc_config.N+1);
    P = SX.sym('P', mpc_config.n_states + mpc_config.n_states);
    
    % Initialize objective and constraints
    obj = 0;
    g = [];
    
    % Initial condition constraint
    g = [g; X(:,1) - P(1:mpc_config.n_states)];
    
    % trajectory over prediction horizon
    for k = 1:mpc_config.N
        st = X(:,k);
        con = U(:,k);
        
        % Stage cost
        state_error = st - P(mpc_config.n_states+1:2*mpc_config.n_states);
        obj = obj + state_error' * mpc_config.Q * state_error + con' * mpc_config.R * con;
        
        % Dynamics constraint using Forward Euler integration
        st_next = X(:,k+1);
        f_value = mpc_config.dynamics_function(st, con);
        st_next_euler = st + mpc_config.dt * f_value;
        g = [g; st_next - st_next_euler];
    end
    
    % Terminal cost
    terminal_error = X(:,end) - P(mpc_config.n_states+1:2*mpc_config.n_states);
    obj = obj + terminal_error' * mpc_config.Qf * terminal_error;
    
    % Create optimization variables vector
    OPT_variables = [reshape(X, mpc_config.n_states*(mpc_config.N+1), 1); 
                     reshape(U, mpc_config.n_controls*mpc_config.N, 1)];
    
    % Create NLP problem
    nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);
    
    % Solver options
    opts = struct;
    opts.ipopt.max_iter = 200;
    opts.ipopt.print_level = 0;
    opts.print_time = 0;
    opts.ipopt.acceptable_tol = 1e-8;
    opts.ipopt.acceptable_obj_change_tol = 1e-6;
    opts.ipopt.warm_start_init_point = 'yes';
    
    % Create solver
    solver = nlpsol('solver', 'ipopt', nlp_prob, opts);
    
    % Set up constraint bounds
    args = setup_constraints(mpc_config, length(g), length(OPT_variables));
    
    % Store in persistent state
    mpc_state.solver = solver;
    mpc_state.args = args;
    mpc_state.prev_solution = args.x0;
    
end