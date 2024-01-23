%% Angle-Axis Trajectory
% Jonathan Woolfrey
%
% This script shows a multi-point trajectory for orientation using the
% cubic spline. Spherical Linear IntERPolation (SLERP) generates
% trajectories for orientation in quaternion space between TWO points.
% Splines in quaternion space are extremely difficult to differentiate,
% hence I resort to angle-axis for more than two points. This script is
% really just to test if angle-axis can be properly interpolated via cubic
% spline.


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

%% Parameters

angles = [0 pi/2 pi 3*pi/4];            % Angles of rotation
axes = [1 1 0 0                         % Axes of rotation
        0 0 1 0
        0 0 0 1];

points = [angles;axes];                 % Concatenate as an array of "points"
times = [0 1 2 3];                      % Time to reach each point

dt = 1/100;                             % Discrete time steps (1/frequency)
steps = (times(end)+1)/dt;              % Number of steps to iterate the loops

R = Rotation('angleAxis',angles(1),axes(:,1));  % Create rotation object of first point

angle = nan(1,steps);                   % Pre-allocate memory
axis = nan(3,steps);                    % Pre-allocate memory

traj = CSpline(times,points);           % Create cubic spline object

for i = 1:steps
    t = (i-1)*dt;                       % Current simulation time
    p = traj.getState(t);               % Get the desired point
    angle(i) = p(1);                    % Angle of rotation
    temp = p(2:4);                      % Axis of rotation
    temp = temp/norm(temp);             % Ensure axis is unit-norm
    axis(:,i) = temp;                   % Assign normalized axis
    R.setAngleAxis(p(1),temp);          % Update the rotation object
    R.plot                              % Visualize it
    figure(1)
end

%%
plot(axis')              % Visual plot of the axes to ensure unit norm

