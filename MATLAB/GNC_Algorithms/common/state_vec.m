function X = state_vec(telemetry)

    pos = telemetry.odometry.position(:);
    vel = telemetry.odometry.velocity(:);
    quat = telemetry.odometry.q(:);
    ang_vel = telemetry.odometry.angular_velocity(:);

    X = [pos; vel; quat; ang_vel];

end