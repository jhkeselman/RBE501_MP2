clear, clc, close all
addpath('../lib');

plotOn = false;
% Create the environment
% g = [0 0 -9.81]; % Gravity Vector [m/s^2]

tiledlayout(3,2);
% Create the robot and display it in the home configuration
nexttile([3,1])
robot = make_robot();
robot.plot(zeros(1,7));
nexttile();
plot([1,2,3,4]);
nexttile();
plot([1,4,9,16]);
nexttile();
plot([1,3,5,7]);

