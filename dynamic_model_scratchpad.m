Mj1 = tdh(0, robot.d(1), robot.a(1), robot.alpha(1));
Mj2 = Mj1 * tdh(0, robot.d(2), robot.a(2), robot.alpha(2));
Mj3 = Mj2 * tdh(0, robot.d(3), robot.a(3), robot.alpha(3));
Mj4 = Mj3 * tdh(0, robot.d(4), robot.a(4), robot.alpha(4));
Mj5 = Mj4 * tdh(0, robot.d(5), robot.a(5), robot.alpha(5));
Mj6 = Mj5 * tdh(0, robot.d(6), robot.a(6), robot.alpha(6));

M1 = Mj1 * [eye(3) [0, 0, 0]'; 0 0 0 1];
M2 = Mj2 * [eye(3) [-0.3638, 0.006, 0.2275]'; 0 0 0 1];
M3 = Mj3 * [eye(3) [-0.0203 -0.0141 0.07]'; 0 0 0 1];
M4 = Mj4 * [eye(3) [0, 0.019, 0]'; 0 0 0 1];  % Second URDF error: should be 0._0_19
M5 = Mj5 * [eye(3) [0 0 0]'; 0 0 0 1];
M6 = Mj6 * [eye(3) 	[0, 0, -0.04125]'; 0 0 0 1];

M01 = M1;
M12 = pinv(pinv(M2)*M1);
M23 = pinv(pinv(M3)*M2);
M34 = pinv(pinv(M4)*M3);
M45 = pinv(pinv(M5)*M4);
M56 = pinv(pinv(M6)*M5);
M67 = eye(4);