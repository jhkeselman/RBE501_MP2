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
    dt = 1e-3;       % time step [s]
    t  = 0 : dt : 0.3; % total time [s]

    params_traj.t = [0 t(end)]; % start and end time of each movement step
    params_traj.time_step = dt;
    params_traj.q = [params.q q];
    params_traj.v = [0 0];
    params_traj.a = [0 0];
    traj = make_trajectory('quintic', params_traj);
    % Generate the joint profiles (position, velocity, and
    % acceleration)
    jointPos_prescribed = traj.q;
    jointVel_prescribed = traj.v;
    jointAcc_prescribed = traj.a;

    jointPos = jointPos_prescribed;
    jointVel = jointVel_prescribed;
    jointAcc = jointAcc_prescribed;
    jointTau = tau_prescribed;
end