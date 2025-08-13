function px4_validate_gps_ned(our_ned, client, config)
%PX4_VALIDATE_GPS_NED Cross-validate GPS→NED conversion against PX4's estimates
%   Compares our GPS→NED conversion results with PX4's sensor fusion estimates
%   to detect coordinate frame issues or significant deviations.

    % Get validation thresholds
    thresholds = get_validation_thresholds(config);
    
    % Get PX4's current position estimates
    px4_ned = get_px4_position_data(client, config);
    if isempty(px4_ned), return; end
    
    % Calculate and check deviations
    [horizontal_diff, vertical_diff, exceeded] = calculate_coordinate_deviations(...
        our_ned, px4_ned, thresholds);
    
    % Display warning if thresholds exceeded
    if exceeded
        display_validation_warning(our_ned, px4_ned, horizontal_diff, vertical_diff, thresholds);
    elseif is_debug_mode(config)
        fprintf('GPS→NED validation: PASS (%.2fm horizontal, %.2fm vertical)\n', ...
                horizontal_diff, vertical_diff);
    end
end