function [Mlist,Glist] = make_dynamics_model(robot)
% MAKE_KINEMATICS_MODEL Creates the dynamics model of the robot
%
% Inputs: None
%robot
% Output: Mlist - 4x4x7 matrix containing all the transformation matrices between consecutive link frames
%         Glist - 6x6x6 matrix containing the spatial inertia matrices of each link

% Link poses when the robot is in the home configuration
% The URDF file describes a robot with the first joint mounted on top of a
% tower. The tower doesn't rotate, it just wears the rest of the robot as a
% hat. Our representation has the first two joints at the origin, so we
% need to translate down by 0.664 m.
%% The URDF can be found here: http://open-robotics.com/urdf-model-of-a-puma-560-robot/

% % Link 1:
% % We ignore the 0.664 m vertical offset and define this frame at our origin
% J1 = eye(4);  % Transform from base frame to joint 1 frame. <origin> of joint 1
% CM1 = eye(4);  % Link frame 1 expressed in frame 1
% M1 = inv(J1) * L01;
% 
% % Link 2:
% % Frames after link 1 are unaffected by the translation, as link frames are
% % defined in their parent's frame
% J12 = [inv(rpy2r(-90, 0, 0)), zeros(3, 1); 0 0 0 1];  % Link frame of joint 2
% CM2 = J12 * [eye(3), -[-0.3638 0.006 0.2275]'; 0 0 0 1];  % I think this is wrong but it is honestly close enough.
% 
% % Link 3:
% J23 = J12 * [eye(3), [0.4318 0 0.1291]'; 0 0 0 1];
% CM3 = J23 * [eye(3), -[-0.0203, -0.0141 0.07]'; 0 0 0 1];  % This one seems more correct
% 
% % Link 4:
% J34 = J23 * [inv(rpy2r(90, 0, 0)), [-0.0203 0 0]'; 0 0 0 1];
% CM4 = J34 * [rpy2r(-90, 0, 0), -[0 0 0.4331]'; 0 0 0 1];  % This one seems more correct

% J2 = [inv(rpy2r(-90, 0, 0)), zeros(3, 1); 0 0 0 1];
% % CM2 = J2 * [J2(1:3,1:3), -[-0.3638 0.006 0.2275]'; 0 0 0 1];
% CM2 = [eye(3), [-0.3638 0.006 0.2275]'; 0 0 0 1];
% R12 = rpy2r(-90, 0, 0);
% P12 = -R12 * [0.13 0.524 0.539]';
% M2 = [rpy2r(-pi, 0, 0), [0.13 0.524 0.539]'; 0 0 0 1];

% We build the M matrices by first finding the frames at each joint. We use
% the DH parameters for this robot because the URDF file is offset an
% unknown amount in the z direction and this messes with our calculations
% for the first joint. The URDF file defines the COM of link i WRT the
% frame of joint i. After calculating the joint matrices, we use them to
% transform the COM frames for each link into vectors in the space frame.
% With all COM frames expressed in a common frame, we can easily find the
% transformations between them. 

Mj1 = tdh(0, robot.d(1), robot.a(1), robot.alpha(1));
Mj2 = Mj1 * tdh(0, robot.d(2), robot.a(2), robot.alpha(2));
Mj3 = Mj2 * tdh(0, robot.d(3), robot.a(3), robot.alpha(3));
Mj4 = Mj3 * tdh(0, robot.d(4), robot.a(4), robot.alpha(4));
Mj5 = Mj4 * tdh(0, robot.d(5), robot.a(5), robot.alpha(5));
Mj6 = Mj5 * tdh(0, robot.d(6), robot.a(6), robot.alpha(6));

M1 = Mj1 * [eye(3) [0, 0, 0]'; 0 0 0 1];
M2 = Mj2 * [eye(3) [-0.3638, 0.006, 0.2275]'; 0 0 0 1];
M3 = Mj3 * [eye(3) [-0.0203 -0.0141 0.07]'; 0 0 0 1];
M4 = Mj4 * [eye(3) [0, 0.019, 0]'; 0 0 0 1];  % Second URDF error: should be 0._0_19, was 0.19 in file
M5 = Mj5 * [eye(3) [0 0 0]'; 0 0 0 1];
M6 = Mj6 * [eye(3) 	[0, 0, -0.04125]'; 0 0 0 1];

M01 = M1;
M12 = pinv(pinv(M2)*M1);
M23 = pinv(pinv(M3)*M2);
M34 = pinv(pinv(M4)*M3);
M45 = pinv(pinv(M5)*M4);
M56 = pinv(pinv(M6)*M5);
M67 = eye(4);

Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67);

%% Spatial Inertia Matrices
% Link 1:
m1 = 0;
Ib1 = diag([0 0.35 0]);
G(:,:,1) = [Ib1, zeros(3); zeros(3), m1 * eye(3)];

% Link 2:
m2 = 17.4;
Ib2 = diag([0.13 0.524 0.539]);
G(:, :, 2) = [Ib2, zeros(3); zeros(3), m2 * eye(3)];

% Link 3:
m3 = 4.8;
Ib3 = diag([0.066 0.086 0.0125]);
G(:, :, 3) = [Ib3, zeros(3); zeros(3), m3 * eye(3)];

% Link 4:
m4 = 0.082;
Ib4 = diag([0.0018 0.0013 0.0018]);
G(:, :, 4) = [Ib4, zeros(3); zeros(3), m4 * eye(3)];

% Link 5:
m5 = 0.34;
Ib5 = diag([0.0003 0.0004 0.0003]);
G(:, :, 5) = [Ib5, zeros(3); zeros(3), m5 * eye(3)];

% Link 6:
m6 = 0.09;
Ib6 = diag([0.0015 0.0015 0.00004]);
G(:, :, 6) = [Ib6, zeros(3); zeros(3), m6 * eye(3)];

Glist = G;
end