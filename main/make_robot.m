function robot = make_robot()
%MAKE_ROBOT Creates the kinematic structure of the robot.
%
%   This is a factory function that creates the robot used in the homework.
%
%   Inputs: none
%
%   Output: robot - the robot structure, created using Peter Corke's
%   robotics toolbox

%% Using RVC Tools so hopefully this works with the URDF we have
%% Create the manipulator
mdl_LWR;
robot = LWR;
fprintf("test");
end

