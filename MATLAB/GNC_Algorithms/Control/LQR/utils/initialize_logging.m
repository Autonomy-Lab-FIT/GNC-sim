function log_data = initialize_logging()
%INITIALIZE_LOGGING Set up data logging structure

    log_data = struct();
    log_data.time = [];
    log_data.state = [];
    log_data.error = [];
    log_data.control = [];
    log_data.commands = [];
    log_data.index = 1;
end

