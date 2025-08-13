function args = setup_constraints(mpc_config, n_constraints, n_variables)
%SETUP_CONSTRAINTS Sets up bounds for states, controls, and equality constraints

    args = struct;
    
    % Equality constraints
    args.lbg = zeros(n_constraints, 1);
    args.ubg = zeros(n_constraints, 1);
    
    % Initialize bounds vectors
    args.lbx = -inf(n_variables, 1);
    args.ubx = inf(n_variables, 1);
    
    % State constraints over prediction horizon
    for i = 1:mpc_config.N+1
        state_idx = (i-1) * mpc_config.n_states;
        
        % Position bounds [px, py, pz]
        args.lbx(state_idx + 1) = mpc_config.pos_bounds(1,1);  % px_min
        args.ubx(state_idx + 1) = mpc_config.pos_bounds(1,2);  % px_max
        args.lbx(state_idx + 2) = mpc_config.pos_bounds(2,1);  % py_min
        args.ubx(state_idx + 2) = mpc_config.pos_bounds(2,2);  % py_max
        args.lbx(state_idx + 3) = mpc_config.pos_bounds(3,1);  % pz_min
        args.ubx(state_idx + 3) = mpc_config.pos_bounds(3,2);  % pz_max
        
        % Velocity bounds [vx, vy, vz]
        args.lbx(state_idx + 4) = mpc_config.vel_bounds(1,1);  % vx_min
        args.ubx(state_idx + 4) = mpc_config.vel_bounds(1,2);  % vx_max
        args.lbx(state_idx + 5) = mpc_config.vel_bounds(2,1);  % vy_min
        args.ubx(state_idx + 5) = mpc_config.vel_bounds(2,2);  % vy_max
        args.lbx(state_idx + 6) = mpc_config.vel_bounds(3,1);  % vz_min
        args.ubx(state_idx + 6) = mpc_config.vel_bounds(3,2);  % vz_max
    end
    
    % Control constraints over horizon
    control_start_idx = mpc_config.n_states * (mpc_config.N + 1);
    for i = 1:mpc_config.N
        control_idx = control_start_idx + (i-1) * mpc_config.n_controls;
        
        % Acceleration bounds [ax, ay, az]
        args.lbx(control_idx + 1) = mpc_config.accel_bounds(1,1);  % ax_min
        args.ubx(control_idx + 1) = mpc_config.accel_bounds(1,2);  % ax_max
        args.lbx(control_idx + 2) = mpc_config.accel_bounds(2,1);  % ay_min
        args.ubx(control_idx + 2) = mpc_config.accel_bounds(2,2);  % ay_max
        args.lbx(control_idx + 3) = mpc_config.accel_bounds(3,1);  % az_min
        args.ubx(control_idx + 3) = mpc_config.accel_bounds(3,2);  % az_max
    end
    
    % Initial guess (hover condition - all zeros)
    args.x0 = zeros(n_variables, 1);
end