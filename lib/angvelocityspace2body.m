function omega_b = angvelocityspace2body(omega_s,R)
    % ANGVELOCITYSPACE2BODY Given a space frame {s} and a rotating body frame {b}, calculate the angular velocity in the body frame.
    %   OMEGA_B = ANGVELOCITYSPACE2BODY(OMEGA_S, R) calculates the angular velocity in the body frame for a given angular velocity OMEGA_S in the space frame and a rotation matrix R between the space and body frames.
    omega_b = R'*omega_s; % Multiplies inverse of R and omega_s
end