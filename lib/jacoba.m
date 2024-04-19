function J_a = jacoba(S,M,q)
    % JACOBA Calculates the analytic jacobian matrix of a manipulator arm.
    %   J_A = JACOBA(S,M,Q) calculates the analytic jacobian for a robot with screw axes S, home configuration matrix M, and joint values Q.
    %
    %   See also JACOB0, FKINE2, SKEW
    J_s = jacob0(S, q); % Calculates the jacobian matrix for the robot.
    T = fkine(S, M, q, 'space'); % Calculates the forward kinematics in the space frame.
    J_a = -skew(T(1:3, 4)) * J_s(1:3, :) + J_s(4:6, :); % Calculates the analytic jacobian.
end