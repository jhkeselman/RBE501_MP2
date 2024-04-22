function [Mlist,Glist] = make_dynamics_model()
% MAKE_KINEMATICS_MODEL Creates the dynamics model of the robot
%
% Inputs: None
%
% Output: Mlist - 4x4x7 matrix containing all the transformation matrices between consecutive link frames
%         Glist - 6x6x6 matrix containing the spatial inertia matrices of each link


%% Create the manipulator
L1 = 0.3; % Lenght of Link 1 [m]
L2 = 0.3; % Lenght of Link 2 [m]
L3 = 0.15; % Lenght of Link 3 [m]
w  = 0.04; % Link Width [m]
l  = 0.04; % Link Depth [m]

% Link poses when the robot is in the home configuration
M01 = [1 0 0 0;
       0 1 0 0;
       0 0 1 L1/2;
       0 0 0 1];
M12 = [1 0 0 0;
       0 0 1 L2/2;
       0 -1 0 L1/2;
       0 0 0 1];
M23 = [1 0 0 0;
       0 0 1 L3/2;
       0 -1 0 L2/2;
       0 0 0 1];
M34 = [0 1 0 0;
       0 0 1 0;
       1 0 0 L3/2;
       0 0 0 1];

Mlist = cat(3, M01, M12, M23, M34);

%% Spatial Inertia Matrices
% *** Define the link inertial properties ***
m1 = 5;   % Mass of Link 1 [kg]
m2 = 1;   % Mass of Link 2 [kg]
m3 = 1;   % Mass of Link 3 [kg]
Ib1 = [(m1*(w^2+L1^2)/12) 0 0;
       0 (m1*(l^2+L1^2)/12) 0;
       0 0 (m1*(l^2+w^2)/12)];
Ib2 = [(m2*(w^2+L2^2)/12) 0 0;
       0 (m2*(l^2+L2^2)/12) 0;
       0 0 (m2*(l^2+w^2)/12)];
Ib3 = [(m3*(w^2+L3^2)/12) 0 0;
       0 (m3*(l^2+L3^2)/12) 0;
       0 0 (m3*(l^2+w^2)/12)];

% Spatial Inertia Matrices
G1 = [Ib1 zeros(3,3);
      zeros(3) m1*eye(3)];
G2 = [Ib2 zeros(3,3);
      zeros(3) m2*eye(3)];
G3 = [Ib3 zeros(3,3);
      zeros(3) m3*eye(3)];

Glist = cat(3, G1, G2, G3);

end