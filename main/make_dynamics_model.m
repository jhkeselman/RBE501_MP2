function [Mlist,Glist] = make_dynamics_model(robot)
% MAKE_KINEMATICS_MODEL Creates the dynamics model of the robot
%
% Inputs: None
%
% Output: Mlist - 4x4x7 matrix containing all the transformation matrices between consecutive link frames
%         Glist - 6x6x6 matrix containing the spatial inertia matrices of each link

%% Link poses when the robot is in the home configuration
[M01, M12, M23, M34, M45, M56, M67] = calculatelinkframes(robot);
Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67);

%% Spatial Inertia Matrices
% *** Define the link inertial properties ***
m1 = 3.7;
m2 = 8.393;
m3 = 2.275;
m4 = 1.219;
m5 = 1.219;
m6 = 0.1879;
Ib1 = [0.010267495893 0 0;
       0 0.010267495893 0;
       0 0 0.00666];
Ib2 = [0.22689067591 0 0;
       0 0.22689067591 0;
       0 0 0.0151074];
Ib3 = [0.049443313556 0 0;
       0 0.049443313556 0;
       0 0 0.004095];
Ib4 = [0.111172755531 0 0;
       0 0.111172755531 0;
       0 0 0.21942];
Ib5 = [0.111172755531 0 0;
       0 0.111172755531 0;
       0 0 0.21942];
Ib6 = [0.0171364731454 0 0;
       0 0.0171364731454 0;
       0 0 0.033822];
G1 = [Ib1 zeros(3);
      zeros(3) m1*eye(3)];
G2 = [Ib2 zeros(3);
      zeros(3) m2*eye(3)];
G3 = [Ib3 zeros(3);
      zeros(3) m3*eye(3)];
G4 = [Ib4 zeros(3);
      zeros(3) m4*eye(3)];
G5 = [Ib5 zeros(3);
      zeros(3) m5*eye(3)];
G6 = [Ib6 zeros(3);
      zeros(3) m6*eye(3)];
Glist = cat(3, G1, G2, G3, G4, G5, G6);

end