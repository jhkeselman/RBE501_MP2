function R = axisangle2rot(omega,theta)
    % AXISANGLE2ROT Calculate the corresponding rotation matrix for given axis and angle.
    %   R = AXISANGLE2ROT(OMEGA, THETA) calculates the rotation matrix of axis omega by angle theta.
    
    s = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0]; % skew-symetric representation of omega
    R = eye(3) + sin(theta)*s + (1-cos(theta))*s^2; % rotation matrix found using Rodrigues' Formula
end