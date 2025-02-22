function [jointPos_actual, jointVel_actual, jointAcc_actual, tau_acc, t_acc] = move_robot(robot, currentQ, targetPt, force)
    % Takes in the robot, the current joints and the target points. charts the
    % path between the two (vectors for position, velocity, acceleration, 
    % torque, and time)

    % pos = [];
    % vel = [];
    % acc = [];
    % tau = [];
    % t_ = [];  
    % addpath('../lib');
    % Create a kinematic model of the robot
    [S,M] = make_kinematics_model(robot);
    n = size(S,2); % read the number of joints
    
    % Create a dynamical model of the robot
    [Mlist,Glist] = make_dynamics_model(robot);
    g = [0 0 -9.81]';

    targetRot = rpy2r(targetPt(4), targetPt(5), targetPt(6), 'deg');
    targetPose = [targetRot, targetPt(1:3)'; 0 0 0 1];
    targetQ = ikine(S, M, currentQ, targetPose);

    tau_acc = [];
    jointPos_acc = [];
    t_acc = [];
    jointAcc_actual = [];
    jointVel_actual = [];
    jointPos_actual = [];
    
    % Detect IK failure
    if isempty(targetQ)
        return;
    end

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
    jointAcc_actual = zeros(n,size(t,2)); % Joint Acceleration (Actual)

    % For each joint
    for ii = 1 : n
        % Calculate a trajectory using a quintic polynomial
        params_traj.t = [0 t(end)]; % start and end time of each movement step
        params_traj.time_step = dt;
        params_traj.q = [currentQ(ii) targetQ(ii)];
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
        Ftip = -g*force;
        Mtip = skew(T(1:3,4))*Ftip;
        wrench = [Mtip', Ftip']';
        Wrench_in_ee = adjoint(T)'*wrench;
        params_rne.Ftip = Wrench_in_ee;
        tau_prescribed(:,ii) = rne(params_rne);

        % Feed the torques to the forward dynamics model and perform one
        % simulation step
        params_fdyn.jointPos = jointPos_actual(:,ii);   
        params_fdyn.jointVel = jointVel_actual(:,ii);
        params_fdyn.tau = tau_prescribed(:,ii);
        params_fdyn.Ftip = Wrench_in_ee;
        jointAcc = fdyn(params_fdyn);

        % Integrate the joint accelerations to get velocity and
        % position
        jointVel_actual(:,ii+1) = dt * jointAcc + jointVel_actual(:,ii);
        jointPos_actual(:,ii+1) = dt * jointVel_actual(:,ii) + jointPos_actual(:,ii);
        jointAcc_actual(:,ii+1) = jointAcc;
    end

    tau_prescribed(:,end) = tau_prescribed(:,end-1);
    
    tau_acc = [tau_acc tau_prescribed];
    jointPos_acc = [jointPos_acc jointPos_actual];
    t_acc = [t_acc t];
end

