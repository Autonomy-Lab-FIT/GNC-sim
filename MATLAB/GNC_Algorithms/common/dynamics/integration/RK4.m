function [XOUT] = RK4(func, x0, t_step, t)
    
    % Inputs
        % func, function to propagate
        % x0 is the initial condition or state
        % t_step is the discrete time step
        % t is the current time
    
    % Output
        % XOUT is the current discrete state at current time t

    k1 = func(t,x0);
    k2 = func(t+t_step/2,x0+t_step/2*k1);
    k3 = func(t+t_step/2,x0+t_step/2*k2);
    k4 = func(t+t_step,x0+t_step*k3);
    phi = 1/6*(k1+2*k2+2*k3+k4);
    XOUT = x0+t_step*phi;

end