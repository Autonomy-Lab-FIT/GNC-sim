function x_dot = drone_dynamics_6_states(t, x)
    % 6-state dynamics: [px, py, pz, vx, vy, vz]'
    params = get_x500_params;

    % Extract states
    pos = x(1:3);
    vel = x(4:6);
    
    % Parameters from your get_x500_params()
    m = params.m;                    
    g = params.g;                 
    K_drag = params.K_drag_linear;         
    
    % Forces (from PX4 model)
    gravity_force = [0; 0; g];          % NED frame
    drag_force = -K_drag * vel;         % Linear drag
    
    % State derivatives
    pos_dot = vel;
    vel_dot = gravity_force + (1/m) * drag_force;  % + unknown thrust (process noise)
    
    x_dot = [pos_dot; vel_dot];
end