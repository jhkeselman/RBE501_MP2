function [jointPos, jointVel, jointAcc, jointTau] = move_robot(params)
    g = [0 0 -9.81]; % Gravity Vector [m/s^2]
    robot = make_robot();
    [S, M] = make_kinematics_model(robot);
    n = size(S,2); % read the number of joints
    [Mlist,Glist] = make_dynamics_model(robot);

    currentT = fkine(S, M, params.q, 'space');
    currentPose = currentT(1:3, 4);
    targetPose = params.target(1:3)';
    q = params.q;
    c = 0;
    lambda = 0.45;
    while norm(targetPose - currentPose) > 1e-6 && c < 1000
        J = jacoba(S,M,q); % jacobian of S at q
        deltaQ = J'*inv(J*J'+lambda^2*eye(3))*(targetPose - currentPose);
        q = q + deltaQ;
        
        % update pose
        currentT = fkine(S, M, q, 'space');
        currentPose = currentT(1:3, 4);
        c = c + 1;
    end

    tau_acc = [];
    jointPos_acc = [];
    t_acc = [];

    dt = 1e-3;       % time step [s]
    t  = 0 : dt : 0.3; % total time [s]

    % Initialize the arrays where we will accumulate the output of the robot
    % dynamics
    jointPos_prescribed = zeros(n,size(t,2)); % Joint Variables (Prescribed)
    jointVel_prescribed = zeros(n,size(t,2)); % Joint Velocities (Prescribed)
    jointAcc_prescribed = zeros(n,size(t,2)); % Joint Accelerations (Prescribed)
    tau_prescribed      = zeros(n,size(t,2)); % Joint Torques

    jointPos_actual = zeros(n,size(t,2)); % Joint Variables (Actual)
    jointVel_actual = zeros(n,size(t,2)); % Joint Velocities (Actual)

    % For each joint
    for i = 1 : n
        % Calculate a trajectory using a quintic polynomial
        params_traj.t = [0 t(end)]; % start and end time of each movement step
        params_traj.time_step = dt;
        params_traj.q = [waypoints(i,jj) waypoints(i,jj+1)];
        params_traj.v = [0 0];
        params_traj.a = [0 0];

        traj = make_trajectory('quintic', params_traj);

        % Generate the joint profiles (position, velocity, and
        % acceleration)
        jointPos_prescribed(i,:) = traj.q;
        jointVel_prescribed(i,:) = traj.v;
        jointAcc_prescribed(i,:) = traj.a;
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

    for i = 1 : size(t,2) - 1
        % Calculate the joint torques using the RNE algorithm
        params_rne.jointPos = jointPos_prescribed(:,i);
        params_rne.jointVel = jointVel_prescribed(:,i);
        params_rne.jointAcc = jointAcc_prescribed(:,i);
        params_rne.Ftip = zeros(6,1); % end effector wrench

        tau_prescribed(:,i) = rne(params_rne);

        % Feed the torques to the forward dynamics model and perform one
        % simulation step
        params_fdyn.jointPos = jointPos_actual(:,i);
        params_fdyn.jointVel = jointVel_actual(:,i);
        params_fdyn.tau = tau_prescribed(:,i);
        params_fdyn.Ftip = zeros(6,1); % end effector wrench

        jointAcc = fdyn(params_fdyn);

        % Integrate the joint accelerations to get velocity and
        % position
        jointVel_actual(:,i+1) = dt * jointAcc + jointVel_actual(:,i);
        jointPos_actual(:,i+1) = dt * jointVel_actual(:,i) + jointPos_actual(:,i);
    end

    tau_prescribed(:,end) = tau_prescribed(:,end-1);
    
    tau_acc = [tau_acc tau_prescribed];
    jointPos_acc = [jointPos_acc jointPos_actual];
    t_acc = [t_acc t+t(end)*(jj-1)];

    jointPos = [];
    jointVel = [];
    jointAcc = [];
    jointTau = [];