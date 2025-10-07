function x_ref = get_reference_state(t, trajectory)

    % index from time
    index = round(t / trajectory.dt) + 1;
    
    % Clamp index to valid range [1, N]
    index = max(1, min(trajectory.N, index));
    
    % reference state column
    x_ref = trajectory.state(:, index);
    
end