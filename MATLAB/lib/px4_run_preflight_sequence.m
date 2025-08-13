function px4_run_preflight_sequence(client, config)

    try
        fprintf('Checking initial drone state...\n');
        initial_telemetry = px4_get_telemetry(client, config);
        
        % Check if drone is already armed
        if isfield(initial_telemetry, 'vehicle_status') && initial_telemetry.vehicle_status.arming_state == 2
            fprintf('Drone already armed - proceeding with current state\n');
        else
            fprintf('Entering offboard mode...\n');
            px4_enter_offboard_mode(client, config);
            pause(2);
            
            fprintf('Arming drone... \n');
            px4_arm_drone(client, config);
            pause(3);
            fprintf('Drone armed\n');
        end
        catch e
        fprintf('Pre-flight sequence failed: %s\n', e.message);
        
    end
end