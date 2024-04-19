function T = fkine(S,M,q,frame)
    % FKINE Calculates the forward kinematics of a robotic arm using the Product of Exponentials formula in either the space or body frame.
    %   T = FKINE(S, M, Q, FRAME) returns the forward kinematics of an arm with screw axes S, homogenous transformation matrix M for home configuration, and joint variables Q, for either the space or body frame as specified by FRAME.
    %   
    %   See also TWIST2HT
    if size(S, 2) ~= length(q) % Checks that S and q are compatible sizes
        error('invalid input dimensions');
    end
    if strcmp(frame, 'space') % Calculation for space frame
        T = twist2ht(S(:, 1), q(1));
        for i = 2:length(q)
            T = T * twist2ht(S(:, i), q(i));
        end
        T = T*M;
    end
    if strcmp(frame, 'body') % Calculation for body frame
        T = twist2ht(S(:, 1), q(1));
        for i = 2:length(q)
            T = T * twist2ht(S(:, i), q(i));
        end
        T = M*T;
    end
end
