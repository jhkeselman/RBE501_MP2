function V_b = twistspace2body(V_s,T)
    % TWISTSPACE2BODY Given a space frame {s} and a moving body frame {b}, calculate the spatial velocity (twist) in the body frame.
    %   V_B = TWISTSPACE2BODY(V_S, T) calculates the spatial velocity twist in the body frame from a given spatial velocity V_S in the space frame and a homogeneous transformation matrix T between the space and body frame.
    %
    %   See also SKEW, INV
    T = inv(T); % Inverse of T to go from body to space frame.
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    skewP = [0 -p(3) p(2);
             p(3) 0 -p(1);
             -p(2) p(1) 0];
    adjT = [R, zeros(3); skewP*R, R]; % Calculates adjoint transform of T
    V_b = adjT*V_s; % Calculates spatial velocity in body frame.
end