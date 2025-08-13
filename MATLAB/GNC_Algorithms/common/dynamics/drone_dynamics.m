function xdot = drone_dynamics(t, x, u, params)

    % Extract parameters
    m = params.mass;                % mass [kg]
    g = params.gravity;             % gravity [m/s^2]
    I = params.inertia;             % inertia matrix [3x3]
    L_roll = params.L_roll;         % roll arm length [m]
    L_pitch = params.L_pitch;       % pitch arm length [m]
    K_drag_linear = params.K_drag_linear;   % linear drag coefficient
    K_drag_angular = params.K_drag_angular; % angular drag coefficient
    T_max = params.T_max;           % maximum thrust per motor [N]
    Q_max = params.Q_max;           % maximum torque per motor [Nm]
    
    % Convert normalized inputs to actual forces and torques
    T1 = T_max * u(1);
    T2 = T_max * u(2);
    T3 = T_max * u(3);
    T4 = T_max * u(4);
    
    % Motor torques (from document: Q_i = Q_max * u_i)
    Q1 = Q_max * u(1);
    Q2 = Q_max * u(2);
    Q3 = Q_max * u(3);
    Q4 = Q_max * u(4);
    
    % Total thrust (in body frame, pointing up = negative z)
    T_total = T1 + T2 + T3 + T4;
    
    % Full Dynamics: [x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz]
    % States: position x(1:3), velocity x(4:6), quaternion x(7:10), angular velocity x(11:13)
    
    pos = x(1:3);
    vel = x(4:6);
    q = x(7:10);              % quaternion [q0, q1, q2, q3]
    omega = x(11:13);         % angular velocity in body frame
    
    % Normalize quaternion
    q = q / norm(q);
    
    % Rotation matrix from body to inertial (from quaternion)
    C_BI = quat_to_dcm(q);
    
    % Forces in body frame
    F_thrust_body = [0; 0; -T_total];  % Negative z = upward in NED
    
    % Transform thrust to inertial frame
    F_thrust_inertial = C_BI * F_thrust_body;
    
    % Other forces in inertial frame
    F_gravity = [0; 0; m*g];
    F_drag = -K_drag_linear * vel;
    
    F_total = F_gravity + F_thrust_inertial + F_drag;
    
    % Linear acceleration
    accel = F_total / m;
    
    % Moments in body frame (exactly from document)
    M_roll = L_roll * (-T1 + T2 + T3 - T4);
    M_pitch = L_pitch * (T1 - T2 + T3 - T4);
    M_yaw = Q1 + Q2 - Q3 - Q4;
    
    M_thrust = [M_roll; M_pitch; M_yaw];
    M_drag = -K_drag_angular * omega;  % Document: M_a,B = -K_Dω * ω_B
    
    M_total = M_thrust + M_drag;
    
    % Angular acceleration
    omega_dot = I \ (M_total - cross(omega, I * omega));
    
    % Quaternion derivative
    omega_quat = [0; omega];  % Pure quaternion from angular velocity
    q_dot = 0.5 * quaternion_multiply(q, omega_quat);
    
    % Assemble state derivative
    xdot = zeros(13,1);
    xdot(1:3) = vel;           % position derivatives
    xdot(4:6) = accel;         % velocity derivatives
    xdot(7:10) = q_dot;        % quaternion derivatives
    xdot(11:13) = omega_dot;   % angular velocity derivatives
            
end



