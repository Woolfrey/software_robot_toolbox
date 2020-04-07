%% Circle Trajectory
% Jonathan Woolfrey
%
% This script demonstrates the circle trajectory object.

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
r = 0.5;                            % Radius of circle
angle = pi/5;                       % Starting angle (in radians)
revs = 6;                           % Number of revolutions to complete (radians)
t0 = 0;                             % Start time
tf = 3;                             % End time

dt = 1/100;
steps = tf/dt;

traj = Circular(t0,tf,angle,revs,r);
centre = Quintic(t0,tf,[-1;0;0],[1;0;0]);

x = nan(1,10);
y = nan(1,10);

for i = 1:steps

    for j = 1:10
        t = (i+j-1)*dt;
        pos = traj.getPoint(t) + centre.lerp(t);
        x(1,j) = pos(1);
        y(1,j) = pos(2);
    end
    scatter(x,y,'k','Filled')
    axis([-2 2 -2 2])
    axis square
    pause(dt)
end