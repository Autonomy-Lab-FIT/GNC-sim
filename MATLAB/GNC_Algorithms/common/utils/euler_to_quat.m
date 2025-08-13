function q = euler_to_quat(roll, pitch, yaw)
    % Convert Euler angles to quaternion
    % Input: roll, pitch, yaw in radians
    % Output: q = [q0, q1, q2, q3]
    
    cr = cos(roll/2);
    sr = sin(roll/2);
    cp = cos(pitch/2);
    sp = sin(pitch/2);
    cy = cos(yaw/2);
    sy = sin(yaw/2);
    
    q = zeros(4,1);
    q(1) = cr*cp*cy + sr*sp*sy;  % q0 (scalar)
    q(2) = sr*cp*cy - cr*sp*sy;  % q1
    q(3) = cr*sp*cy + sr*cp*sy;  % q2  
    q(4) = cr*cp*sy - sr*sp*cy;  % q3
end