function config = px4_monitor_gps_quality(telemetry, config)
%PX4_MONITOR_GPS_QUALITY Monitor GPS quality transitions during flight
%   Tracks GPS quality degradation and recovery during flight operations.

    % Validate input and initialize monitoring
    if ~validate_monitoring_input(telemetry), return; end
    config = initialize_quality_monitoring(config);
    
    % Extract GPS quality metrics
    gps_metrics = extract_gps_quality_metrics(telemetry.gps);
    
    % Determine current quality status
    [current_state, quality_good] = assess_current_gps_quality(gps_metrics, config);
    
    % Check for state transitions and handle logging
    last_state = config.gps.quality_state.last_state;
    if ~isempty(last_state) && ~strcmp(last_state, current_state)
        handle_quality_transition(current_state, gps_metrics, config);
    end
    
    % Update state tracking
    config.gps.quality_state.last_state = current_state;
    config.gps.quality_state.last_timestamp = datestr(now, 'HH:MM:SS');
end