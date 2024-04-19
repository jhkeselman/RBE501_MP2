function J = jacob0(S,q) 
    % JACOB0 Calculates the Jacobian matrix for a robotic arm (in the space frame).
    %   J = JACOB0(S, Q) calculates the Jacobian for a kinematic chain with screw axes S and joint variables Q.
    %   
    %   See also ADJOINT, TWIST2HT

    J = zeros(6, size(S,2)); % initializes jacobian as array of zeros
    J(:,1) = S(:,1); % first column of jacobian is just first screw axis
    for i = 2:size(S,2) % loop over all screw axes
        T = twist2ht(S(:,1), q(1));
        for ii = 2:i
            T = T*twist2ht(S(:,ii),q(ii)); % calculate twist for each screw axis
        end
        J(:,i) = adjointT(S(:,i), T); % calculate jacobian for each screw axis
    end
end