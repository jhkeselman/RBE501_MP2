function q = ikine(S, M, q_current, target_P)
% Solves inverse kinematics for the given robot
%   S set of screw axes
%   M home configuration matrix
%   current_q current joint configuration
%   target_P HT matrix representing goal pose

    % Initialize current position
    P_current = fkine(S, M, q_current, 'space');
    P_current = MatrixLog6(P_current);
    P_current = [P_current(3,2) P_current(1,3) P_current(2,1) P_current(1:3,4)']';

    % Initialize target position
    P_d = MatrixLog6(target_P);
    P_d = [P_d(3,2) P_d(1,3) P_d(2,1) P_d(1:3,4)']';
    
    for i=1:10
        iterations = 0;
        % starting_lambda = 10;
        starting_lambda = 1;
        lambda = starting_lambda;
        while norm(P_current - P_d) > 0.0001 && iterations < 10000
            iterations = iterations + 1;
            % Calculate Jacobian
            J = jacob0(S, q_current);
    
            % Levenberg Marquardt
            delta_q = J' * pinv(J * J' + lambda * eye(size(J))) * (P_d - P_current);

            % Reject step if delta is too large
            % if norm(delta_q) > 0.1
            if norm(delta_q) > 0.2
                'Rejecting step';
                lambda = lambda * 1.5;
                q_current;
            else
                q_current = q_current + delta_q';
            end
            % 
            % If delta is absurdly small, decrease lambda
            if norm(delta_q) < 1e-5
                'Decreasing lambda';
                lambda = lambda * 0.9;
            end
            
    
            % Update current pose
            P_current = fkine(S, M, q_current, 'space');
            P_current = MatrixLog6(P_current);
            P_current = [P_current(3,2) P_current(1,3) P_current(2,1) P_current(1:3,4)']';
    
            % norm(P_current - P_d);
            % pause(0.5);
        end

        if norm(P_current - P_d) < 0.0001
            q = q_current;
            return;
        else
            % If solution not found, pick a random starting point and go
            % again
            q_current = pi * rand(1, 6) - pi / 2;
            "Going again";
            dist = norm(P_current - P_d);
            norm_delta = norm(delta_q);
        end
    end
    
    q = [];
    warning("IK Failed")
end

