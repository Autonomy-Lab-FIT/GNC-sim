function q_result = quaternion_multiply(q1, q2)
    % Multiplying two quaternions
    % q = [q0, q1, q2, q3] where q0 is scalar part
    
    q1_0 = q1(1); q1_vec = q1(2:4);
    q2_0 = q2(1); q2_vec = q2(2:4);
    
    q_result = zeros(4,1);
    q_result(1) = q1_0*q2_0 - dot(q1_vec, q2_vec);
    q_result(2:4) = q1_0*q2_vec + q2_0*q1_vec + cross(q1_vec, q2_vec);
end