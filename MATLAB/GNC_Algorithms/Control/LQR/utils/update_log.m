function log_data = update_log(log_data, t, x_current, x_error, u_lqr)
%UPDATE_LOG Add current data to log

    i = log_data.index;
    
    log_data.time(i) = t;
    log_data.state(:,i) = x_current;
    log_data.error(:,i) = x_error;
    log_data.control(:,i) = u_lqr;
    % log_data.commands.thrust(i) = attitude_cmd.thrust;
    % log_data.commands.roll_rate(i) = attitude_cmd.roll_rate;
    % log_data.commands.pitch_rate(i) = attitude_cmd.pitch_rate;
    % log_data.commands.yaw_rate(i) = attitude_cmd.yaw_rate;
    
    log_data.index = i + 1;
end

