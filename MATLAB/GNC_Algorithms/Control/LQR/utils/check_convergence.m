function [converged, status_msg] = check_convergence(x_error, threshold, t)
%CHECK_CONVERGENCE Check if system has converged to target

    pos_error = norm(x_error(1:3));
    vel_error = norm(x_error(4:6));
    total_error = pos_error + vel_error;
    
    converged = total_error < threshold;
    
    status_msg = sprintf('Pos: %.2fm, Vel: %.2fm/s', pos_error, vel_error);
end