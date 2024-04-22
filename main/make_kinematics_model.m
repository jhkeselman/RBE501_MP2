function [S,M] = make_kinematics_model()
% MAKE_KINEMATICS_MODEL Calculates the Screw Axes and Home Configuration of
% the robot.
%
% Inputs: robot - the robot object created by the robotics toolbox
%
% Output: S - 6xn matrix whose columns are the screw axes of the robot
%         M - homogeneous transformation representing the home configuration

% Screw Axes
S = [0 0 1 0 0 0;
     0 -1 0 0.2 0 0;
     0 0 1 0 0 0;
     0 1 0 -0.2 0 0;
     0 0 1 0 0 0;
     0 -1 0.19 0 0 0;
     0 0 1 0 0 0]';

% Home configuration
R_home = [1 0 0; 
          0 1 0; 
          0 0 1]';
t_home = [0 0 0.078]';
M = [R_home t_home; 0 0 0 1];

end