function [S,M] = make_kinematics_model(robot)
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
M = double(robot.fkine(zeros(1,6)));

end

%% Should all be good. using RVC tools for home config, screw axes from joints in URDF