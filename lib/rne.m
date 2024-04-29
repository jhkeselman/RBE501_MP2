function [tau,V,Vdot] = rne(params)
    % RNE Implements the Recursive Newton-Euler Inverse Dynamics Algorithm
    %
    % Inputs: params - a structure containing the following fields:
    %           params.g - 3-dimensional column vector describing the acceleration of gravity
    %           params.S - 6xn matrix of screw axes (each column is an axis)
    %           params.M - 4x4xn home configuration matrix for each link
    %           params.G - 6x6xn spatial inertia matrix for each link
    %           params.jointPos - n-dimensional column vector of joint coordinates
    %           params.jointVel - n-dimensional column vector of joint velocities
    %           params.jointAcc - n-dimensional column vector of joint accelerations
    %           params.Ftip - 6-dimensional column vector representing the
    %           wrench applied at the tip
    %
    % Output: tau  - n-dimensional column vector of generalized joint forces
    %         V    - 6x(n+1) matrix - each column represents the twist of one of the robot's links
    %         Vdot - 6x(n+1) matrix - each column represents the acceleration of one of the robot's links
    %
    % Forward iterations
    
    % some code based on office hours with professor on 4/15/24
    
    g = params.g;
    S = params.S;
    M = params.M;
    G = params.G;
    Ftip = params.Ftip;
    jointPos = params.jointPos;
    jointVel = params.jointVel;
    jointAcc = params.jointAcc;
    
    [~,n] = size(params.S); % number of joints
    A = zeros(6,n);
    T = zeros(4,4,n);
    M_ = eye(4);
    adT = zeros(6,6,n+1);
    adT(:,:,n+1) = adjoint(pinv(M(:,:,n+1)));
    V = zeros(6,n+1);
    Vdot = zeros(6,n+1);
    Vdot(4:6,1) = -g;
    
    % Calculate the home configurations of each link, expressed in the space frame  
    for i = 1:n
        M_ = M_ * M(:,:,i);
        A(:,i) = adjoint(pinv(M_)) * S(:,i);
        T(:,:,i) = twist2ht(A(:,i),-jointPos(i)) * pinv(M(:,:,i)); % 8.50
        
        adT(:,:,i) = adjoint(T(:,:,i));
        V(:,i+1) = adT(:,:,i) * V(:,i) + A(:,i) * jointVel(i); % 8.51
        
        Vdot(:,i+1) = adT(:,:,i) * Vdot(:,i) + ad(V(:,i+1)) * A(:,i) * jointVel(i) + A(:,i) * jointAcc(i); % 8.52
    end
    
    % Backward iterations
    tau = zeros(n,1);
    F = Ftip;
    for i = n:-1:1
        F = adT(:,:,i+1)' * F + G(:,:,i) * Vdot(:,i+1) - ad(V(:,i+1))' * (G(:,:,i) * V(:,i+1)); % 8.53
        tau(i) = F'*A(:,i); % 8.54
    end
end