function analyze_results(filename)
    %ANALYZE_HOVER_RESULTS Generate plots and RMSE from logged data (works for both hover and waypoint tests)
    load(filename, 'data');
    
    % Use only the valid data
    n = data.actual_steps;
    time = data.time(1:n);
    ukf = data.ukf_states(1:n, :);
    gps = data.gps_ned(1:n, :);
    ekf2 = data.ekf2_states(1:n, :);
    
    % Get commanded trajectory (should always exist now)
    commanded = [data.commanded_x(1:n), data.commanded_y(1:n), data.commanded_z(1:n)];
    
    if isfield(data, 'trajectory_type')
        fprintf('\n=== %s TRAJECTORY TEST RESULTS ===\n', upper(data.trajectory_type));
    else
        fprintf('\n=== HOVER TEST RESULTS ===\n');
    end
    
    % === POSITION ANALYSIS ===
    figure('Name', 'Position Comparison');
    subplot(3,1,1); % North
    plot(time, ukf(:,1), 'b-', 'LineWidth', 2); hold on;
    plot(time, gps(:,1), 'r--', 'LineWidth', 1);
    plot(time, ekf2(:,1), 'g:', 'LineWidth', 1.5);
    plot(time, commanded(:,1), 'k--', 'LineWidth', 1, 'Color', [0.5 0.5 0.5]);
    ylabel('North (m)'); legend('UKF', 'GPS', 'EKF2', 'Commanded'); grid on;
    
    subplot(3,1,2); % East
    plot(time, ukf(:,2), 'b-', 'LineWidth', 2); hold on;
    plot(time, gps(:,2), 'r--', 'LineWidth', 1);
    plot(time, ekf2(:,2), 'g:', 'LineWidth', 1.5);
    plot(time, commanded(:,2), 'k--', 'LineWidth', 1, 'Color', [0.5 0.5 0.5]);
    ylabel('East (m)'); legend('UKF', 'GPS', 'EKF2', 'Commanded'); grid on;
    
    subplot(3,1,3); % Down
    plot(time, ukf(:,3), 'b-', 'LineWidth', 2); hold on;
    plot(time, gps(:,3), 'r--', 'LineWidth', 1);
    plot(time, ekf2(:,3), 'g:', 'LineWidth', 1.5);
    plot(time, commanded(:,3), 'k--', 'LineWidth', 1, 'Color', [0.5 0.5 0.5]);
    ylabel('Down (m)'); xlabel('Time (s)'); legend('UKF', 'GPS', 'EKF2', 'Commanded'); grid on;
    
    % === VELOCITY ANALYSIS ===
    figure('Name', 'Velocity Comparison');
    subplot(3,1,1); % North velocity
    plot(time, ukf(:,4), 'b-', 'LineWidth', 2); hold on;
    plot(time, gps(:,4), 'r--', 'LineWidth', 1);
    plot(time, ekf2(:,4), 'g:', 'LineWidth', 1.5);
    ylabel('V_N (m/s)'); legend('UKF', 'GPS', 'EKF2'); grid on;
    
    subplot(3,1,2); % East velocity
    plot(time, ukf(:,5), 'b-', 'LineWidth', 2); hold on;
    plot(time, gps(:,5), 'r--', 'LineWidth', 1);
    plot(time, ekf2(:,5), 'g:', 'LineWidth', 1.5);
    ylabel('V_E (m/s)'); legend('UKF', 'GPS', 'EKF2'); grid on;
    
    subplot(3,1,3); % Down velocity
    plot(time, ukf(:,6), 'b-', 'LineWidth', 2); hold on;
    plot(time, gps(:,6), 'r--', 'LineWidth', 1);
    plot(time, ekf2(:,6), 'g:', 'LineWidth', 1.5);
    ylabel('V_D (m/s)'); xlabel('Time (s)'); legend('UKF', 'GPS', 'EKF2'); grid on;
    
    % === 2D TRAJECTORY PLOT (for all tests) ===
    figure('Name', '2D Trajectory View');
    plot(commanded(:,1), commanded(:,2), 'k--', 'LineWidth', 2); hold on;
    plot(ukf(:,1), ukf(:,2), 'b-', 'LineWidth', 2);
    plot(gps(:,1), gps(:,2), 'r:', 'LineWidth', 1.5);
    plot(ekf2(:,1), ekf2(:,2), 'g:', 'LineWidth', 1.5);
    
    % Mark start and end points
    plot(commanded(1,1), commanded(1,2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot(commanded(end,1), commanded(end,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    xlabel('East (m)'); ylabel('North (m)');
    legend('Commanded', 'UKF', 'GPS', 'EKF2', 'Start', 'End');
    title('Flight Path');
    grid on; axis equal;
    
    % === PATH FOLLOWING ERROR PLOT ===
    figure('Name', 'Path Following Error');
    ukf_path_error = sqrt(sum((ukf(:,1:3) - commanded).^2, 2));
    gps_path_error = sqrt(sum((gps(:,1:3) - commanded).^2, 2));
    ekf2_path_error = sqrt(sum((ekf2(:,1:3) - commanded).^2, 2));
    
    plot(time, ukf_path_error, 'b-', 'LineWidth', 2); hold on;
    plot(time, gps_path_error, 'r--', 'LineWidth', 1.5);
    plot(time, ekf2_path_error, 'g:', 'LineWidth', 1.5);
    
    xlabel('Time (s)'); ylabel('Error from Commanded (m)');
    legend('UKF', 'GPS', 'EKF2'); grid on;
    title('Distance from Commanded Position');
    
    % === RMSE CALCULATION ===
    % UKF vs EKF2 (truth reference)
    pos_error_ukf = ukf(:,1:3) - ekf2(:,1:3);
    vel_error_ukf = ukf(:,4:6) - ekf2(:,4:6);
    rmse_pos_ukf = sqrt(mean(pos_error_ukf.^2));
    rmse_vel_ukf = sqrt(mean(vel_error_ukf.^2));
    
    % GPS vs EKF2 (baseline comparison)
    pos_error_gps = gps(:,1:3) - ekf2(:,1:3);
    vel_error_gps = gps(:,4:6) - ekf2(:,4:6);
    rmse_pos_gps = sqrt(mean(pos_error_gps.^2));
    rmse_vel_gps = sqrt(mean(vel_error_gps.^2));
    
    % === TRAJECTORY METRICS ===
    % Path following errors
    ukf_path_error_mean = mean(sqrt(sum((ukf(:,1:3) - commanded).^2, 2)));
    gps_path_error_mean = mean(sqrt(sum((gps(:,1:3) - commanded).^2, 2)));
    ekf2_path_error_mean = mean(sqrt(sum((ekf2(:,1:3) - commanded).^2, 2)));
    
    ukf_path_error_max = max(sqrt(sum((ukf(:,1:3) - commanded).^2, 2)));
    gps_path_error_max = max(sqrt(sum((gps(:,1:3) - commanded).^2, 2)));
    ekf2_path_error_max = max(sqrt(sum((ekf2(:,1:3) - commanded).^2, 2)));
    
    % === RESULTS SUMMARY ===
    fprintf('Test Duration: %.1f seconds\n', time(end));
    fprintf('Data Points: %d\n', n);
    
    fprintf('\nPosition RMSE (vs EKF2):\n');
    fprintf(' UKF: [%.3f, %.3f, %.3f] m\n', rmse_pos_ukf);
    fprintf(' GPS: [%.3f, %.3f, %.3f] m\n', rmse_pos_gps);
    
    fprintf('\nVelocity RMSE (vs EKF2):\n');
    fprintf(' UKF: [%.3f, %.3f, %.3f] m/s\n', rmse_vel_ukf);
    fprintf(' GPS: [%.3f, %.3f, %.3f] m/s\n', rmse_vel_gps);
    
    fprintf('\nOverall Position RMSE:\n');
    fprintf(' UKF: %.3f m\n', norm(rmse_pos_ukf));
    fprintf(' GPS: %.3f m\n', norm(rmse_pos_gps));
    
    fprintf('\nPath Following Performance:\n');
    fprintf(' UKF Mean Error: %.3f m (Max: %.3f m)\n', ukf_path_error_mean, ukf_path_error_max);
    fprintf(' GPS Mean Error: %.3f m (Max: %.3f m)\n', gps_path_error_mean, gps_path_error_max);
    fprintf(' EKF2 Mean Error: %.3f m (Max: %.3f m)\n', ekf2_path_error_mean, ekf2_path_error_max);

    fprintf('\n=== END ===\n');
end