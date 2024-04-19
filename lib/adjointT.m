function twist_inB = adjointT(twist_inA,T_AB)
    % ADJOINT Calculates the representation of a twist axis in an arbitrary reference frame.
    %   TWIST_INB = ADJOINT(TWIST_INA, T_AB) returns the representation of twist TWIST_INA in a new reference frame described by homogeneous transformation matrix T_AB.
    
    R = T_AB(1:3, 1:3);
    p = T_AB(1:3, 4);
    s = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
    ad = [R zeros(3); s*R R]; % adjoint from T_AB
    twist_inB = ad*twist_inA; % multiplies adjoint by twist axis
end

