function xdot = drone_nonlinear_dynamics(t, x, u, params)
    
    
    % drone parameters
    m = params.m;
    g = params.g;
    J = params.inertia;
    
    Jx = J(1,1);
    Jy = J(2,2);
    Jz = J(3,3);
    
    % states
    r = x(1:3);
    v = x(4:6);
    q = x(7:10);
    omega = x(11:13);
    
    % controls
    U1 = u(1);
    U2 = u(2);
    U3 = u(3);
    U4 = u(4);
    
    % quaternions
    q0 = q(1);
    q1 = q(2);
    q2 = q(3);
    q3 = q(4);
    
    % angular rates
    p_rate = omega(1);
    q_rate = omega(2);
    r_rate = omega(3);
    
    rdot = v;
    
    vxdot = -(U1/m) * 2*(q1*q3 + q0*q2);
    vydot = -(U1/m) * 2*(q2*q3 - q0*q1);
    vzdot = g - (U1/m) * (q0^2 - q1^2 - q2^2 + q3^2);
    vdot = [vxdot; vydot; vzdot];
    
    q0dot = 0.5 * (-p_rate*q1 - q_rate*q2 - r_rate*q3);
    q1dot = 0.5 * (p_rate*q0 + r_rate*q2 - q_rate*q3);
    q2dot = 0.5 * (q_rate*q0 - r_rate*q1 + p_rate*q3);
    q3dot = 0.5 * (r_rate*q0 + q_rate*q1 - p_rate*q2);
    qdot = [q0dot; q1dot; q2dot; q3dot];
    
    pdot = (1/Jx) * (U2 + (Jy - Jz)*q_rate*r_rate);
    qrdot = (1/Jy) * (U3 + (Jz - Jx)*p_rate*r_rate);
    rdot_ang = (1/Jz) * (U4 + (Jx - Jy)*p_rate*q_rate);
    omegadot = [pdot; qrdot; rdot_ang];
    
    xdot = [rdot; vdot; qdot; omegadot];
end