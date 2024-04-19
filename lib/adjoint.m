function AdT = adjoint(T)
    % ADJOINT Given an homogeneous transformation matrix T, calculate the corresponding 6x6 adjoint transformation matrix.
    %   ADT = ADJOINT(T) calculates the corresponding 6x6 adjoint transformation matrix for matrix T.
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    s = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
    AdT = [R zeros(3); s*R R];
end