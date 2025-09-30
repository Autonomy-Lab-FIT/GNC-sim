function display_status(t, dt, status_msg)
%DISPLAY_STATUS Print control status at regular intervals

    if mod(round(t/dt), 20) == 0  % Every 1 second
        fprintf('[t=%.1fs] %s\n', t, status_msg);
    end
end