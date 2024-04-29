function [tau,V,Vdot] = rne(params)
%% RNE Implements the Recursive Newton-Euler Inverse Dynamics Algorithm
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
    n = size(params.S, 2);
    A = zeros(6,n);
    V = zeros(6, n+1);
    Vdot = zeros(6,n+1);
    Vdot(:,1) = [0 0 0 -params.g]';
    M = zeros(4,4,n);
    M(:,:,1) = params.M(:,:,1);
    for i = 2:n+1
        M(:,:,i) = M(:,:,i-1)*params.M(:,:,i);
    end
    for i = 1:n
        s = params.S(:,i);
        m = params.M(:,:,i);
        A(:,i) = adjoint(inv(M(:,:,i))) * s;
        j = params.jointPos(i);
        t = twist2ht(-A(:,i), j);
        T = t*inv(m);
        V(:,i+1) = A(:,i) * params.jointVel(i) + adjoint(T)*V(:,i);
        Vdot(:,i+1) = A(:,i) * params.jointAcc(i) + adjoint(T)*Vdot(:,i) + ad(V(:,i+1))*A(:,i)*params.jointVel(i);
    end
    
% Backward iterations
% YOUR CODE HERE
    tau = zeros(n,1);
    W = zeros(6,n+1);
    W(:,n+1) = params.Ftip(:);
    for i = n:-1:1
        if i == n
            T = inv(params.M(:,:,n+1));
        else
            j = params.jointPos(i+1);
            t = twist2ht(-A(:,i+1), j);
            T = t*inv(params.M(:,:,i+1));
        end
        G = params.G(:,:,i);
        vdot = Vdot(:,i+1);
        v = V(:,i+1);
        W(:,i) = G*vdot  - ad(v)'*G*v + adjoint(T)'*W(:,i+1);
        tau(i) = W(:,i)'*A(:,i);
    end
end