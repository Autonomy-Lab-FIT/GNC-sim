function args = setup_constraints_with_obstacles(mpc_config, num_g, num_opt_vars)
%SETUP_CONSTRAINTS_WITH_OBSTACLES Defines the bounds for the optimization problem
    args.lbx = -inf(num_opt_vars, 1);
    args.ubx = inf(num_opt_vars, 1);
    args.x0 = zeros(num_opt_vars, 1);
    
    % Set state and control bounds
    n_states = mpc_config.n_states;
    n_controls = mpc_config.n_controls;
    N = mpc_config.N;

    for k = 1:N+1
        offset = (k-1)*n_states;
        args.lbx(offset+1:offset+3) = mpc_config.pos_bounds(:,1);
        args.ubx(offset+1:offset+3) = mpc_config.pos_bounds(:,2);
        args.lbx(offset+4:offset+6) = mpc_config.vel_bounds(:,1);
        args.ubx(offset+4:offset+6) = mpc_config.vel_bounds(:,2);
    end

    for k = 1:N
        offset = n_states*(N+1) + (k-1)*n_controls;
        args.lbx(offset+1:offset+3) = mpc_config.accel_bounds(:,1);
        args.ubx(offset+1:offset+3) = mpc_config.accel_bounds(:,2);
    end

    % Set constraint bounds (lbg <= g(x,p) <= ubg)
    args.lbg = zeros(num_g, 1);
    args.ubg = zeros(num_g, 1);

    % Obstacle constraints are one-sided (>=)
    num_dyn_constraints = n_states * (N + 1);
    args.ubg(num_dyn_constraints+1:end) = inf;
end