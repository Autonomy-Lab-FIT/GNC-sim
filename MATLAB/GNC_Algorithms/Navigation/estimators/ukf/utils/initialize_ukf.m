function [x0, P0] = initialize_ukf(telemetry, config, ukf_config)
    
    % Get initial GPS position/velocity
    [north, east, down] = geodetic_to_ned(telemetry.gps.latitude_deg, ...
                                      telemetry.gps.longitude_deg, ...
                                      telemetry.gps.altitude_msl_m, ...
                                      config.ref_lat, ...
                                      config.ref_lon, ...
                                      config.ref_alt);
    % down = -down;
    vel_ned = [telemetry.gps.vel_n_m_s; 
               telemetry.gps.vel_e_m_s; 
               telemetry.gps.vel_d_m_s];
    
    % Initial state
    x0 = [north; east; down; vel_ned];
    
    % Initial covariance
    P0 = ukf_config.P0;
end