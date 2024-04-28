function [Mlist,Glist] = make_dynamics_model()
% MAKE_KINEMATICS_MODEL Creates the dynamics model of the robot
%
% Inputs: None
%
% Output: Mlist - 4x4x7 matrix containing all the transformation matrices between consecutive link frames
%         Glist - 6x6x6 matrix containing the spatial inertia matrices of each link

%% Should be right number of links, but no clue how to get these actual values
% Link poses when the robot is in the home configuration
% The URDF file describes a robot with the first joint mounted on top of a
% tower. The tower doesn't rotate, it just wears the rest of the robot as a
% hat. Our representation has the first two joints at the origin, so we
% need to translate down by 0.664 m. We also put our robot in a different
% starting orientation than the URDF, so we need to apply a rotation of
% pi/2 about joint 2.  

% Link 1:
M1 = eye(3)
% Transformation step 1: Translate link down by 0.664m

M01 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
M12 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
M23 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
M34 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
M45 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
M56 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
M67 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
M78 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];

Mlist = cat(8, M01, M12, M23, M34, M45, M56, M67, M78);

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