function T = tdh(theta, d, a, alpha)
    % TDH Calculates the transformation matrix for a set of DH parameters.
    %   T = TDH(THETA, D, A, ALPHA) returns the transformation matrix T for a set of DH parameters.
    T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta); 
        sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta); 
        0 sin(alpha) cos(alpha) d; 
        0 0 0 1];
end