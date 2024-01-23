%% Test Cubic Spline
% Jonathan Woolfrey
%
% This script tests the generation of cubic spline trajectories.

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
m = 3;                                          % Dimensions (use 3 to plot the results)
n = 10;                                          % Go crazy, try 100!
points = rand(m,n);                             % Randomly generate a series of points
times = 1:n;                                    % Time to reach each point                 
         
%% Set Up Arrays and Generate the Trajectory
dt = 1/100;                                     % Time step (1/frequency)
steps = (times(end)+1)/dt;                      % Total steps to run simulation
pos = nan(m,steps);                             % Save position information
vel = nan(m,steps);                             % Save velocity information
acc = nan(m,steps);                             % Save acceleration information
traj = CSpline(times,points);                   % Create trajectory object

tic                                             % Start the timer
for i = 1:steps
    t = (i-1)*dt;                               % Current simulation time
    [pos(:,i),vel(:,i),acc(:,i)] = traj.getState(t); % Get current state
end
toc/steps                                       % Average computation time

%% Plot Results
t = (0:steps-1)*dt;                         % Time array

figure(1)
subplot(3,1,1)
    plot(t,pos,'LineWidth',1)               % Plot trajectories
    hold on
        for i = 1:m
            scatter(times,points(i,:),'filled') % Overlay waypoints
        end
    hold off
    box off
    ylabel('Position')
subplot(3,1,2)
    plot(t,vel,'LineWidth',1)               % Plot velocities
    box off
    ylabel('Velocity')
subplot(3,1,3)
    plot(t,acc,'LineWidth',1)               % Plot accelerations
    box off
    ylabel('Acceleration')
    xlabel('Time (s)')
set(gcf,'Color',[1 1 1])

if m == 3                                   % 3D data
    figure(2)
    plot3(pos(1,:),pos(2,:),pos(3,:),'k','LineWidth',1) % Plot trajectory
    hold on
    scatter3(points(1,:),points(2,:),points(3,:),'k','filled')
    hold off
    grid on
    box off
    axis equal
    set(gcf,'Color',[1 1 1])
end