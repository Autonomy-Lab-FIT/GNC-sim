function save_log_data(log_data, filename)
%SAVE_LOG_DATA Save logged data to file

    log_data.index = [];  % Remove index field
    save(filename, 'log_data');
    fprintf('Control log saved to: %s\n', filename);
end