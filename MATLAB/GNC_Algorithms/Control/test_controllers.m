% Connect to your PX4 bridge
config = px4_get_config();
client = px4_connect(config.ip_address, config.port);

% Run the hover position control test (P)
% run_hover_position_control_test(config, client);

% Run the hover velocity control test (PID)
% run_hover_cascade_controller_test(config, client);

run_LQR(config, client)

 % Cleanup
px4_cleanup(client);
