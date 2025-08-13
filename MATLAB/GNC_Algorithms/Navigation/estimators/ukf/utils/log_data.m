function data = log_data(data, step, t, x_ukf, telemetry, config)
    % Store time
    data.time(step) = t;
    
    % Store UKF states
    data.ukf_states(step, :) = x_ukf';
    %fprintf('LOG CALL: step=%d, t=%.1f, ukf_z=%.3f\n', step, t, x_ukf(3));
    % Convert GPS to NED and store
    [north, east, down] = geodetic_to_ned(telemetry.gps.latitude_deg, ...
                                          telemetry.gps.longitude_deg, ...
                                          telemetry.gps.altitude_msl_m, ...
                                          config.ref_lat, ...
                                          config.ref_lon, ...
                                          config.ref_alt);
    %down = -down;
    gps_vel = [telemetry.gps.vel_n_m_s; 
               telemetry.gps.vel_e_m_s; 
               telemetry.gps.vel_d_m_s];
    
    data.gps_ned(step, :) = [north, east, down, gps_vel'];
    
    % Store PX4 EKF2 estimates
    ekf2_pos = [telemetry.local_position.x;
                telemetry.local_position.y; 
                telemetry.local_position.z];
                
    ekf2_vel = [telemetry.local_position.vx;
                telemetry.local_position.vy;
                telemetry.local_position.vz];
    
    data.ekf2_states(step, :) = [ekf2_pos', ekf2_vel'];
    
    data.valid_steps = step;
end