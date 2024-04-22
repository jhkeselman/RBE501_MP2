function [jointPos, jointVel, jointAcc, jointTau] = move_robot(params)
    robot = make_robot();
    [S, N] = make_kinematics_model(robot);
    n = size(S,2); % read the number of joints
    [Mlist,Glist] = make_dynamics_model(robot);
    jointPos = [];
    jointVel = [];
    jointAcc = [];
    jointTau = [];
end