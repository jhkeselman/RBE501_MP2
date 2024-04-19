function T = twist2ht(S,theta)
    % TWIST2HT Calculates the corresponding homogeneous transformation matrix for a given twist axis and angle.
    %   T = TWIST2HT(S, THETA) returns the homogeneous matrix T for a twist S and angle THETA.
    %   
    %   See also AXISANGLE2ROT
    
    if numel(S) ~= 6 % check that S is a valid screw axis vector
        error('S is not a valid screw axis vector');
    end
    w = S(1:3); % angular velocity
    v = S(4:6);
    s = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0]; % skew-symetric of w
    R = axisangle2rot(w, theta); % rotation matrix of omega by theta
    t = (eye(3)*theta + (1-cos(theta))*s + (theta-sin(theta))*s^2)*v;
    T = [R, t; 0 0 0 1]; % final transformation matrix
end