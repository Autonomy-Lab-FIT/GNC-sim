function mpc_config = mpc_dynamics_model(mpc_config)
%MPC_DYNAMICS_MODEL Creates symbolic dynamics model using CasADi
%   Uses same dynamics as UKF with Euler integration

    import casadi.*
    
    % Get drone parameters (same as UKF)
    params = get_x500_params();
    m = params.mass;
    g = params.gravity;
    K_drag = params.K_drag_linear;
    
    % Define symbolic variables
    px = SX.sym('px'); py = SX.sym('py'); pz = SX.sym('pz');
    vx = SX.sym('vx'); vy = SX.sym('vy'); vz = SX.sym('vz');
    states = [px; py; pz; vx; vy; vz];
    mpc_config.n_states = length(states);
    
    % Control inputs (acceleration commands)
    ax = SX.sym('ax'); ay = SX.sym('ay'); az = SX.sym('az');
    controls = [ax; ay; az];
    mpc_config.n_controls = length(controls);
    
    % Dynamics equations
    pos = states(1:3);
    vel = states(4:6);
    
    % Forces
    gravity_force = [0; 0; g];      % NED frame gravity
    drag_force = -K_drag * vel;     % Linear drag
    
    % State derivatives
    pos_dot = vel;                                              % Position derivative
    vel_dot = gravity_force + (1/m) * drag_force + controls;    % Velocity derivative
    
    % Complete state derivative vector
    rhs = [pos_dot; vel_dot];
    
    % Create CasADi dynamics function
    mpc_config.dynamics_function = Function('f', {states, controls}, {rhs});
    mpc_config.params = params;
   
end