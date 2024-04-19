function J_b = jacobe(S,M,q)    
    % JACOBE Calculates the jacobian matrix in the end effector frame.
    %   J_B = JACOBE(S,M,Q) calculates the Jacobian for a kinematic chain with screw axes S, home matrix M, and joint variables Q.
    %
    %   See also JACOB0, FKINE2, INV
    J0 = jacob0(S,q); % Calculates the jacobian matrix.
    T = fkine2(S, M, q, 'space'); % Calculates the forward kinematics in the space frame.
    T = inv(T);
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    skewP = [0 -p(3) p(2);
             p(3) 0 -p(1);
             -p(2) p(1) 0];
    adjT = [R, zeros(3); skewP*R, R]; % Calculates the adjoint transform of the matrix T.
    J_b = adjT*J0; % Multiples the adjoint and the jacobian to get the end-effector jacobian.
end