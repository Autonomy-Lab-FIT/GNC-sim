function [K, u_eq] = compute_k_matrix(A, B, config_lqr, px4_config)
    
    weights = config_lqr.weights;
    control_weights = config_lqr.control_weights;
    
    % Check controllability
    % Co = ctrb(A, B);
    % fprintf('Controllability matrix rank: %d (should be 13)\n', rank(Co));


    Q = diag([weights.position * ones(1,3), ...      % x, y, z
              weights.velocity * ones(1,3), ...      % vx, vy, vz  
              weights.att * ones(1,4), ...      % phi, theta, psi
              weights.ang_vel * ones(1,3)]); % wx, wy, wz

    R = diag(control_weights);

    K = lqr(A, B, Q, R);

    u_eq = [px4_config.m*px4_config.g; 0; 0; 0];

    % fprintf('LQR gains: K = [%.3f %.3f %.3f %.3f] (showing first 4 elements)\n', K(1,1:4));

end

