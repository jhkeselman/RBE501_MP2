function [Mlist,Glist] = make_dynamics_model()
% MAKE_KINEMATICS_MODEL Creates the dynamics model of the robot
%
% Inputs: None
%
% Output: Mlist - 4x4x7 matrix containing all the transformation matrices between consecutive link frames
%         Glist - 6x6x6 matrix containing the spatial inertia matrices of each link

%% Should be right number of links, but no clue how to get these actual values
% Link poses when the robot is in the home configuration
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

%% This is also all placeholders
%% Spatial Inertia Matrices
% *** Define the link inertial properties ***
m1 = 1;   % Mass of Link 1 [kg]
m2 = 1;   % Mass of Link 2 [kg]
m3 = 1;   % Mass of Link 3 [kg]
m4 = 1;   % Mass of Link 4 [kg]
m5 = 1;   % Mass of Link 5 [kg]
m6 = 1;   % Mass of Link 6 [kg]
m7 = 1;   % Mass of Link 7 [kg]
Ib1 = [1 0 0;
       0 1 0;
       0 0 1];
Ib2 = [1 0 0;
       0 1 0;
       0 0 1];
Ib3 = [1 0 0;
       0 1 0;
       0 0 1];
Ib4 = [1 0 0;
       0 1 0;
       0 0 1];
Ib5 = [1 0 0;
       0 1 0;
       0 0 1];
Ib6 = [1 0 0;
       0 1 0;
       0 0 1];
Ib7 = [1 0 0;
       0 1 0;
       0 0 1];

% Spatial Inertia Matrices
G1 = [Ib1 zeros(3,3);
      zeros(3) m1*eye(3)];
G2 = [Ib2 zeros(3,3);
      zeros(3) m2*eye(3)];
G3 = [Ib3 zeros(3,3);
      zeros(3) m3*eye(3)];
G4 = [Ib4 zeros(3,3);
      zeros(3) m4*eye(3)];
G5 = [Ib5 zeros(3,3);
      zeros(3) m5*eye(3)];
G6 = [Ib6 zeros(3,3);
      zeros(3) m6*eye(3)];
G7 = [Ib7 zeros(3,3);
      zeros(3) m7*eye(3)];

Glist = cat(7, G1, G2, G3, G4, G5, G6, G7);

end