function tau = move_robot(robot, params)
% This will take in a set of parameters from the GUI including the robot,
% the desired pose and wrench, and returns all the relevant info for
% plotting the robot and the graphs. we can probably move this to live in
% the GUI if we want (or just keep it here for visual ease)
    x = params.x;
    y = params.y;
    z = params.z;
    roll = params.roll;
    pitch = params.pitch;
    yaw = params.yaw;
    wrench = params.wrench;
    lx = params.lx;
    ly = params.ly;
    lz = params.lz;
    lroll = params.lroll;
    lpitch = params.lpitch;
    tau = [];
end