function S = skew(w)
    % SKEW Convert a 3x1 vector to the corresponding skew-symetric matrix.
    %   S = SKEW(W) converts vector W to skew-symetric matrix S.
    
    S = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0]; % skew-symetric representation of omega
end