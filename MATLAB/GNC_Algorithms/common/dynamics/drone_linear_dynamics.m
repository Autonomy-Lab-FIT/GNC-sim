function [A, B] = drone_linear_dynamics(x_curr, u_curr, params)
    
    % Linearizing the dynamics at the current state using Taylor series
    m = params.m;
    J = params.inertia;

    q_curr = x_curr(7:10);
    omega_curr = x_curr(11:13);
    U1_curr = u_curr(1);

    q0_curr = q_curr(1);
    q1_curr = q_curr(2);
    q2_curr = q_curr(3);
    q3_curr = q_curr(4);

    p_curr = omega_curr(1);
    q_curr_ang = omega_curr(2);
    r_curr = omega_curr(3);

    e = [0; 0; 1];
    
    R_curr = [1-2*(q2_curr^2+q3_curr^2),   2*(q1_curr*q2_curr-q0_curr*q3_curr),   2*(q1_curr*q3_curr+q0_curr*q2_curr);
          2*(q1_curr*q2_curr+q0_curr*q3_curr),   1-2*(q1_curr^2+q3_curr^2),   2*(q2_curr*q3_curr-q0_curr*q1_curr);
          2*(q1_curr*q3_curr-q0_curr*q2_curr),   2*(q2_curr*q3_curr+q0_curr*q1_curr),   1-2*(q1_curr^2+q2_curr^2)];

    % compute the Jacobian
    s = e;
    Jq = zeros(3, 4);

    Jq(:,1) = 2*q0_curr*s + 2*(cross([q1_curr; q2_curr; q3_curr],s));
    
    qv_curr = [q1_curr; q2_curr; q3_curr];
    S_s = [0, -s(3), s(2); s(3), 0, -s(1); -s(2), s(1), 0]; %skew matrix of s

    Jq(:,2:4) = 2*((qv_curr'*s)*eye(3) + qv_curr*s' - s*qv_curr' - q0_curr*S_s);

    Omega_curr = [0, -p_curr, -q_curr_ang, -r_curr;
                   p_curr, 0, r_curr, -q_curr_ang;
                   q_curr_ang, -r_curr, 0, p_curr;
                   r_curr, q_curr_ang, -p_curr, 0];

    dOmega_dp = [0, 0, 0, 0;
                 1, 0, 0, 0;
                 0, 0, 0, 1;
                 0, 0, -1, 0];

    dOmega_dq = [0, 0, -1, 0;
                 0, 0, 0, -1;
                 1, 0, 0, 0;
                 0, 1, 0, 0];

    dOmega_dr = [0, 0, 0, -1;
                 0, 0, 1, 0;
                 0, -1, 0, 0;
                 1, 0, 0, 0];

    Aq_omega = 0.5 * [dOmega_dp*q_curr, dOmega_dq*q_curr, dOmega_dr*q_curr];

    J_omega_curr = J * omega_curr;
    S_J_omega_curr = [0, -J_omega_curr(3), J_omega_curr(2);
                      J_omega_curr(3), 0, -J_omega_curr(1);
                      -J_omega_curr(2), J_omega_curr(1), 0];
    
    S_omega_curr = [0, -omega_curr(3), omega_curr(2);
                    omega_curr(3), 0, -omega_curr(1);
                    -omega_curr(2), omega_curr(1), 0];
    
    M_omega = S_J_omega_curr - S_omega_curr * J;
    A_omega_omega = J \ M_omega;
    
    % A matrix (13x13)
    % States: [r(3); v(3); q(4); omega(3)]
    A = zeros(13, 13);
    
    % Position: dr/dt = v
    A(1:3, 4:6) = eye(3);
    
    % Velocity: dv/dt 
    % drag=0
    A(4:6, 7:10) = -(U1_curr/m) * Jq;
    
    % Quaternion: dq/dt
    A(7:10, 7:10) = 0.5 * Omega_curr;
    A(7:10, 11:13) = Aq_omega;
    A(7, 7) = -1e-6;

    % Angular rate: domega/dt
    A(11:13, 11:13) = A_omega_omega;
    
    % B matrix (13x4)
    B = zeros(13, 4);
    
    % U1
    B(4:6, 1) = -(1/m) * R_curr * e;
    
    % U2, U3, U4
    B(11:13, 2:4) = J \ eye(3);
end