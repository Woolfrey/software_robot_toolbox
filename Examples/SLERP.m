%% Spherical Linear IntERPolation
% Jonathan Woolfrey
%
% Orientation in 3D space can be represented by a unit quaternion. Since
% the coordinates specified by the quaternion lie on the surface of a unit
% 4D sphere, SLERP is required to generate trajectories across said
% surface. 
%
% This script was developed to test (and demonstrate) that SLERP works
% correctly for the trapezoidal and quintic polynomial trajectory objects.


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

%%

p1 = rand(4,1);                     % First quaternion
p2 = rand(4,1);                     % Second quaternion

p1 = p1/norm(p1);                   % Normalize
p2 = p2/norm(p2);                   % Normalize

dt = 1/100;                         % Discrete time step

t0 = 1;                             % Start time
tf = 5;                             % End time

steps = (tf+1)/dt;                  % No. of steps to iterate through

traj = Trapezoidal(t0,tf,p1,p2);    % Create trajectory object

% These are for recording data
pos = nan(4,steps);
vel = nan(3,steps);
acc = nan(3,steps);
s = nan(1,steps);
sd = nan(1,steps);
sdd = nan(1,steps);

for i = 1:steps
    t = (i-1)*dt;                                   % Current simulation time
    [s(i), sd(i), sdd(i)] = traj.getScalar(t);      % Interpolation scalar
    [pos(:,i),vel(:,i),acc(:,i)] = traj.slerp(t);   % Current state
end

%% Plot results

subplot(3,1,1)
plot(s)
subplot(3,1,2)
plot(sd)
subplot(3,1,3)
plot(sdd)

figure(2)
plot(vel')
    