function trajectory = generate_circular_trajectory(params)

    radius = params.radius;
    altitude = params.altitude;
    period = params.period;
    start_angle = params.start_angle;
    dt = params.dt;
    duration = params.duration;

    N = ceil(duration / dt) + 1;
    time_vector = linspace(0, duration, N);
    
    % Pre-allocate state matrix [13 x N]
    state = zeros(13, N);

    omega =  2 * pi/period; %rad/s
    
    for i = 1:N
        t = time_vector(i);
        theta = omega * t + start_angle;
        % position
        x = radius * cos(theta);
        y = radius * sin(theta);
        z = altitude;
    
        % velocity
        vx = -radius * omega * sin(theta);
        vy = radius * omega * cos(theta);
        vz = 0;
    
        % attitude, assume level-flight
        roll = 0;
        pitch = 0;
        yaw = atan2(vy, vx);
        
        quat = eul2quat([roll, pitch, yaw], 'ZYX');
    
        % angular velocity
        p = 0;
        q = 0;
        r = omega;
    
        % reference state vector
        state(1, i) = x;
        state(2, i) = y;
        state(3, i) = z;
        state(4, i) = vx;
        state(5, i) = vy;
        state(6, i) = vz;
        state(7, i) = quat(1);  % q0
        state(8, i) = quat(2);  % q1
        state(9, i) = quat(3);  % q2
        state(10, i) = quat(4); % q3
        state(11, i) = p;
        state(12, i) = q;
        state(13, i) = r;
    end

    trajectory = struct(...
        'time_vector', time_vector, ...
        'state', state, ...
        'dt', dt, ...
        'N', N);

end