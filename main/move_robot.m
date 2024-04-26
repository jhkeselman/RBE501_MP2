function R = move_robot(params)
    x = params.x;
    y = params.y;
    z = params.z;
    q = [x y z 0 0 0 0];
    robot = make_robot();
    robot.plot(q);
    R = [];
end