function R = move_robot(params)
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
    lyaw = params.lyaw;
    robot = make_robot();
    q1 = [x y z roll pitch yaw 0];
    q2 = [lx ly lz lroll lpitch lyaw 0];
    q = [q1' q2'];
    robot.plot(q(:,1:2:end)');
    R = [];
end