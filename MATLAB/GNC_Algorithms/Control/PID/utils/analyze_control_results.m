function analyze_control_results(filename, control_type)
%ANALYZE_CONTROL_RESULTS Analyze control test results
%   Inputs:
%       filename     - Path to saved data file
%       control_type - 'position', 'cascade', or 'mpc'
    
    load(filename, 'data');
    
    
    valid_idx = 1:data.actual_steps;
    hover_target = [0, 0, -2]; % Target position
    
    % Calculate RMSE metrics
    pos_rmse_x = sqrt(mean((data.ukf_pos_x(valid_idx) - hover_target(1)).^2));
    pos_rmse_y = sqrt(mean((data.ukf_pos_y(valid_idx) - hover_target(2)).^2));
    pos_rmse_z = sqrt(mean((data.ukf_pos_z(valid_idx) - hover_target(3)).^2));
    pos_rmse_3d = sqrt(mean((data.ukf_pos_x(valid_idx) - hover_target(1)).^2 + ...
                           (data.ukf_pos_y(valid_idx) - hover_target(2)).^2 + ...
                           (data.ukf_pos_z(valid_idx) - hover_target(3)).^2));
    
    vel_rmse_x = sqrt(mean(data.ukf_vel_x(valid_idx).^2));
    vel_rmse_y = sqrt(mean(data.ukf_vel_y(valid_idx).^2));
    vel_rmse_z = sqrt(mean(data.ukf_vel_z(valid_idx).^2));
    vel_rmse_3d = sqrt(mean(data.ukf_vel_x(valid_idx).^2 + data.ukf_vel_y(valid_idx).^2 + data.ukf_vel_z(valid_idx).^2));
    
    % Display results
    fprintf('\n=== %s Control Analysis ===\n', upper(control_type));
    fprintf('Position RMSE: X=%.3fm, Y=%.3fm, Z=%.3fm, 3D=%.3fm\n', pos_rmse_x, pos_rmse_y, pos_rmse_z, pos_rmse_3d);
    fprintf('Velocity RMSE: X=%.3fm/s, Y=%.3fm/s, Z=%.3fm/s, 3D=%.3fm/s\n', vel_rmse_x, vel_rmse_y, vel_rmse_z, vel_rmse_3d);
    
    % MPC performance
    if strcmp(control_type, 'mpc') && isfield(data, 'mpc_solve_time')
        valid_solve_times = data.mpc_solve_time(valid_idx);
        valid_solve_times = valid_solve_times(~isnan(valid_solve_times));
        if ~isempty(valid_solve_times)
            fprintf('MPC Solve Time: Mean=%.1fms, Max=%.1fms\n', ...
                    mean(valid_solve_times), max(valid_solve_times));
        end
    end
    fprintf('================================\n');
    
    % Create plots
    figure('Name', sprintf('%s Control Analysis', upper(control_type)), 'Position', [100, 100, 1200, 800]);
    
    
    if strcmp(control_type, 'cascade') || strcmp(control_type, 'mpc')
        subplot_rows = 2; subplot_cols = 4;
    else
        subplot_rows = 2; subplot_cols = 3;
    end
    
    % Plot 1: Position tracking
    subplot(subplot_rows, subplot_cols, 1);
    plot(data.time(valid_idx), data.ukf_pos_x(valid_idx), 'b-', 'LineWidth', 1.5); hold on;
    plot(data.time(valid_idx), data.ukf_pos_y(valid_idx), 'r-', 'LineWidth', 1.5);
    plot(data.time(valid_idx), data.ukf_pos_z(valid_idx), 'g-', 'LineWidth', 1.5);
    yline(hover_target(1), 'b--', 'X Target');
    yline(hover_target(2), 'r--', 'Y Target');
    yline(hover_target(3), 'g--', 'Z Target');
    xlabel('Time (s)'); ylabel('Position (m)'); title('Position Tracking');
    legend('X', 'Y', 'Z', 'Location', 'best'); grid on;
    
    % Plot 2: Position errors 
    subplot(subplot_rows, subplot_cols, 2);
    plot(data.time(valid_idx), data.ukf_pos_x(valid_idx) - hover_target(1), 'b-', 'LineWidth', 1.5); hold on;
    plot(data.time(valid_idx), data.ukf_pos_y(valid_idx) - hover_target(2), 'r-', 'LineWidth', 1.5);
    plot(data.time(valid_idx), data.ukf_pos_z(valid_idx) - hover_target(3), 'g-', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('Position Error (m)'); 
    title(sprintf('Position Errors (3D RMSE: %.3fm)', pos_rmse_3d));
    legend('X Error', 'Y Error', 'Z Error', 'Location', 'best'); grid on;
    
    % Plot 3: Velocity estimates
    subplot(subplot_rows, subplot_cols, 3);
    plot(data.time(valid_idx), data.ukf_vel_x(valid_idx), 'b-', 'LineWidth', 1.5); hold on;
    plot(data.time(valid_idx), data.ukf_vel_y(valid_idx), 'r-', 'LineWidth', 1.5);
    plot(data.time(valid_idx), data.ukf_vel_z(valid_idx), 'g-', 'LineWidth', 1.5);
    yline(0, 'k--', 'Zero');
    xlabel('Time (s)'); ylabel('Velocity (m/s)'); 
    title(sprintf('Velocity Estimates (3D RMSE: %.3fm/s)', vel_rmse_3d));
    legend('Vx', 'Vy', 'Vz', 'Location', 'best'); grid on;
    
    % Plot 4: Velocity commands 
    subplot(subplot_rows, subplot_cols, 4);
    plot(data.time(valid_idx), data.vel_cmd_x(valid_idx), 'b-', 'LineWidth', 1.5); hold on;
    plot(data.time(valid_idx), data.vel_cmd_y(valid_idx), 'r-', 'LineWidth', 1.5);
    plot(data.time(valid_idx), data.vel_cmd_z(valid_idx), 'g-', 'LineWidth', 1.5);
    yline(0, 'k--', 'Zero');
    xlabel('Time (s)'); ylabel('Velocity Command (m/s)'); title('Position Controller Output');
    legend('Vel Cmd X', 'Vel Cmd Y', 'Vel Cmd Z', 'Location', 'best'); grid on;
    
    % Cascade and MPC both have acceleration commands
    if strcmp(control_type, 'cascade') || strcmp(control_type, 'mpc')
        % Plot 5: Acceleration commands
        subplot(subplot_rows, subplot_cols, 5);
        plot(data.time(valid_idx), data.accel_cmd_x(valid_idx), 'b-', 'LineWidth', 1.5); hold on;
        plot(data.time(valid_idx), data.accel_cmd_y(valid_idx), 'r-', 'LineWidth', 1.5);
        plot(data.time(valid_idx), data.accel_cmd_z(valid_idx), 'g-', 'LineWidth', 1.5);
        yline(0, 'k--', 'Zero');
        xlabel('Time (s)'); ylabel('Acceleration Command (m/s²)'); 
        title(sprintf('%s Controller Output', upper(control_type)));
        legend('Accel Cmd X', 'Accel Cmd Y', 'Accel Cmd Z', 'Location', 'best'); grid on;
        
        % Plot 6: 3D trajectory
        subplot(subplot_rows, subplot_cols, 6);
        plot3(data.ukf_pos_x(valid_idx), data.ukf_pos_y(valid_idx), -data.ukf_pos_z(valid_idx), 'b-', 'LineWidth', 1.5); hold on;
        plot3(hover_target(1), hover_target(2), -hover_target(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        xlabel('North (m)'); ylabel('East (m)'); zlabel('Up (m)'); title('3D Trajectory');
        legend('UKF Trajectory', 'Target', 'Location', 'best'); grid on; axis equal;
    else
        % Position controller: 3D trajectory
        subplot(subplot_rows, subplot_cols, 5);
        plot3(data.ukf_pos_x(valid_idx), data.ukf_pos_y(valid_idx), -data.ukf_pos_z(valid_idx), 'b-', 'LineWidth', 1.5); hold on;
        plot3(hover_target(1), hover_target(2), -hover_target(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        xlabel('North (m)'); ylabel('East (m)'); zlabel('Up (m)'); title('3D Trajectory');
        legend('UKF Trajectory', 'Target', 'Location', 'best'); grid on; axis equal;
    end
    
    sgtitle(sprintf('%s Control Performance Analysis', upper(control_type)), 'FontSize', 14, 'FontWeight', 'bold');
    
    % Save plots
    saveas(gcf, strrep(filename, '.mat', '_analysis.fig'));
    saveas(gcf, strrep(filename, '.mat', '_analysis.png'));
    
    fprintf('Analysis plots saved as %s\n', strrep(filename, '.mat', '_analysis.fig/.png'));
end