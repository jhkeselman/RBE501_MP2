function [jointPos_acc, jointVel_actual, jointAcc, tau_acc, t_acc] = move_robot(robot, currentQ, targetPt, force)
    % Takes in the robot, the current joints and the target points. charts the
    % path between the two (vectors for position, velocity, acceleration, 
    % torque, and time)

    % pos = [];
    % vel = [];
    % acc = [];
    % tau = [];
    % t_ = [];

    % Create a kinematic model of the robot
    [S,M] = make_kinematics_model(robot);
    n = size(S,2); % read the number of joints
    
    % Create a dynamical model of the robot
    [Mlist,Glist] = make_dynamics_model(robot);
    g = [0 0 -9.81]';

    
    targetCoords = targetPt(1:3)'; % [x y z]
    currentCoords = fkine(S,M,currentQ,'space');
    currentCoords = currentCoords(1:3,4);
    currentQ_ = currentQ; % store current position
    error = targetCoords - currentCoords;
    lambda = 0.1;
    while norm(error) > 1e-3
        J_a = jacoba(S,M,currentQ_);
        % deltaQ = pinv(J_a) * error; % pseudoinverse
        deltaQ = J_a' * pinv(J_a * J_a' + lambda^2 * eye(3)) * error; % DLS
        % alpha = dot(error,J_a*J_a'*error)/dot(J_a*J_a'*error,J_a*J_a'*error);
        % deltaQ = alpha * J_a' * error; % transpose
        currentQ_ = currentQ_ + deltaQ';
        currentCoords = fkine(S,M,currentQ_,'space');
        currentCoords = currentCoords(1:3,4);
        error = targetCoords - currentCoords;
    end
    targetQ = currentQ_;

    tau_acc = [];
    jointPos_acc = [];
    t_acc = [];
       
    % Initialize the time vector
    dt = 1e-3;       % time step [s]
    t  = 0 : dt : 0.5; % total time [s]

    % Initialize the arrays where we will accumulate the output of the robot
    % dynamics
    jointPos_prescribed = zeros(n,size(t,2)); % Joint Variables (Prescribed)
    jointVel_prescribed = zeros(n,size(t,2)); % Joint Velocities (Prescribed)
    jointAcc_prescribed = zeros(n,size(t,2)); % Joint Accelerations (Prescribed)
    tau_prescribed      = zeros(n,size(t,2)); % Joint Torques

    jointPos_actual = zeros(n,size(t,2)); % Joint Variables (Actual)
    jointVel_actual = zeros(n,size(t,2)); % Joint Velocities (Actual)

    % For each joint
    for ii = 1 : n
        % Calculate a trajectory using a quintic polynomial
        params_traj.t = [0 t(end)]; % start and end time of each movement step
        params_traj.time_step = dt;
        params_traj.q = [currentQ targetQ];
        params_traj.v = [0 0];
        params_traj.a = [0 0];

        traj = make_trajectory('quintic', params_traj);

        % Generate the joint profiles (position, velocity, and
        % acceleration)
        jointPos_prescribed(ii,:) = traj.q;
        jointVel_prescribed(ii,:) = traj.v;
        jointAcc_prescribed(ii,:) = traj.a;
    end

    % Initialize the parameters for both inverse and forward dynamics
    params_rne.g = g; % gravity
    params_rne.S = S; % screw axes
    params_rne.M = Mlist; % link frames
    params_rne.G = Glist; % inertial properties
    params_fdyn.g = g; % gravity
    params_fdyn.S = S; % screw axes
    params_fdyn.M = Mlist; % link frames
    params_fdyn.G = Glist; % inertial properties


    % Initialize the (actual) joint variables
    jointPos_actual(:,1) = jointPos_prescribed(:,1);
    jointVel_actual(:,1) = jointVel_actual(:,1);

    for ii = 1 : size(t,2) - 1
        % Calculate the joint torques using the RNE algorithm
        params_rne.jointPos = jointPos_prescribed(:,ii);
        params_rne.jointVel = jointVel_prescribed(:,ii);
        params_rne.jointAcc = jointAcc_prescribed(:,ii);
        % params_rne.Ftip = zeros(6,1); % end effector wrench
        T = fkine(S,M,params_rne.jointPos,'space');
        Ftip_inS = [cross(T(1:3,4),-g);-g*force];
        params_rne.Ftip = adjoint(T)' * Ftip_inS;
        tau_prescribed(:,ii) = rne(params_rne);

        % Feed the torques to the forward dynamics model and perform one
        % simulation step
        params_fdyn.jointPos = jointPos_actual(:,ii);
        params_fdyn.jointVel = jointVel_actual(:,ii);
        params_fdyn.tau = tau_prescribed(:,ii);
        params_fdyn.Ftip = zeros(6,1); % end effector wrench
        T = fkine(S,M,params_fdyn.jointPos,'space');
        Ftip_inS = [cross(T(1:3,4),-g);-g*force];
        params_fdyn.Ftip = adjoint(T)' * Ftip_inS;
        jointAcc = fdyn(params_fdyn);

        % Integrate the joint accelerations to get velocity and
        % position
        jointVel_actual(:,ii+1) = dt * jointAcc + jointVel_actual(:,ii);
        jointPos_actual(:,ii+1) = dt * jointVel_actual(:,ii) + jointPos_actual(:,ii);
    end

    tau_prescribed(:,end) = tau_prescribed(:,end-1);
    
    tau_acc = [tau_acc tau_prescribed];
    jointPos_acc = [jointPos_acc jointPos_actual];
    t_acc = [t_acc t]; % TODO
% t_acc
% jointPos_acc
% jointPos_actual
% tau_acc
end

