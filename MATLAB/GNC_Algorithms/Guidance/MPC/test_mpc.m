% Connect to your PX4 bridge
config = px4_get_config();
client = px4_connect(config.ip_address, config.port);

% Run the point stabilization test for mpc
run_mpc_point_stabilization_test(config, client);

 % Cleanup
px4_cleanup(client);