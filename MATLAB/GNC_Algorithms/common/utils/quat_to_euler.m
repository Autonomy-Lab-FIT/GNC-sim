function [roll, pitch, yaw] = quat_to_euler(q)
    % Convert quaternion to Euler angles (ZYX convention)
    % Input: q = [q0, q1, q2, q3]
    % Output: roll, pitch, yaw in radians
    
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    
    % Roll (x-axis rotation)
    sinr_cosp = 2 * (q0 * q1 + q2 * q3);
    cosr_cosp = 1 - 2 * (q1^2 + q2^2);
    roll = atan2(sinr_cosp, cosr_cosp);
    
    % Pitch (y-axis rotation)  
    sinp = 2 * (q0 * q2 - q3 * q1);
    if abs(sinp) >= 1
        pitch = sign(sinp) * pi/2;
    else
        pitch = asin(sinp);
    end
    
    % Yaw (z-axis rotation)
    siny_cosp = 2 * (q0 * q3 + q1 * q2);
    cosy_cosp = 1 - 2 * (q2^2 + q3^2);
    yaw = atan2(siny_cosp, cosy_cosp);
end