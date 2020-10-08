%% Inverse Kinematics
% Jon Woolfrey
%
% I created this script to test the functionality of the inverse kinematics
% solver(s) that I wrote.


% Copyright (C) Jon Woolfrey, 2019-2020
% 
% This file is part of the Robot Toolbox I developed for MATLAB.
%
% My Robot Toolbox is free software and may be distributed and/or modified
% according to the terms of the GNU General Public Licence v3.0
% (https://www.gnu.org/licenses/gpl-3.0.en.html). A copy should be included
% in the root directory.
%
% I developed this toolbox to simulate sophisticated robot control methods
% for my research, which other packages were lacking. I hope others may
% find it useful so they don't have to endure the same pains I did.
%
% This software is made available without warranty, fitness for use, or
% merchantability. If any public works are distributed that were made
% possible because of this Robot Toolbox, a citation or reference would be
% much appreciated!
%
% jonathan.woolfrey@gmail.com

close all
clear all
clc

load ur3.mat;
%% 

p = [0.25;-0.1;0.3];                     % Desired end-effector positions
R = Rotation('rpy',[pi/2, 0, pi/2]);    % Desired end-effector orientation
P = Pose(p,R);                          % Combine as a Pose object


% % This configuration passes through itself
% q0 = [0.2889    0.9841   -0.0704   -0.6040    0.3686   -0.1712]';

q0 = randn(6,1);                        % Initial guess for IK solver

q = robot.ik(P,q0,'method','transpose'); % Method is either 'transpose' or 'inverse'

robot.plot3D(q,'workspace',[-0.2 0.8 -0.5 0.5 0 1]) % Plot the end result