%% Pick-and-Place Task
% Jon Woolfrey
%
% This script demonstrates a "pick-and-place" task for a robot arm. Given
% any number of desired end-effector poses, the inverse kinematics is
% solved for each. This creates an array of discrete joint configurations.
%
% A trajectory is then created to tell the robot how to transition between
% these discrete configurations.
%
% Finally, feedback control is applied to track this trajectory.
%
% A plot of the joint trajectories and joint states is produced at the end
% to examine performance.



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

%% Parameters
animate = 1;
method = 1;                                                                 % 1 = PD+, 2 = Inverse dynamics
dt = 1/100;                                                                 % Inverse of control frequency
steps = 15/dt;                                                              % Total no. of steps to run simulation
workspace = [-0.2 0.8 -0.5 0.5 0 1];                                        % Workspace for plotting robot

%% Generate End-Effector Poses

% Set waypoint positions
p = nan(3,3);
p(:,1) = [0.3; 0.2; 0.1];
p(:,2) = [0.3; -0.1; 0.4];
p(:,3) = [0.3; -0.1; 0.1];

% Set waypoint orientations as rotation objects
R(1,3) = Rotation();
R(1) = Rotation('rpy',[-pi, 0, pi/2]);
R(2) = Rotation('rpy',[pi/2, 0, -pi/2]);
R(3) = Rotation('rpy',[pi, 0, -pi/2]);

% Create Pose object for each waypoint
P(1,3) = Pose();
for i = 1:3
    P(i) = Pose(p(:,i),R(i));
end

%% Solve Inverse Kinematics
qd = zeros(6,4);                                                            % Waypoints to reach
q0 = zeros(6,1);                                                            % Initial joint configuration
qd(:,1) = robot.ik(P(1),q0,'method','transpose');                           % First waypoint
qd(:,2) = robot.ik(P(2),qd(:,1)+0.05*ones(6,1),'method','inverse');         % Second waypoint
qd(:,3) = robot.ik(P(3),qd(:,2),'method','inverse');                        % Third waypoint
qd(:,4) = qd(:,1);                                                          % Start where we finished

%% Create trajectory object
times = [2 6 9 13];                                                         % Time to reach each waypoint
trajectory = CSpline(times,qd);                                             % Create cubic spline

%% Set up initial values & arrays
q = qd(:,1);                                                                % Initial joint configuration
qdot = zeros(6,1);                                                          % Initial joint velocities
qddot = zeros(6,1);                                                         % Initial joint accelerations

desiredState = nan(6,steps,3);                                              % For plotting data
jointState = nan(6,steps,3);                                                % For plotting data

%% Run simulation
tic
for i = 1:steps
    
    % Update robot state
    q = q + dt*qdot + 0.5*dt*dt*qddot;          
    qdot = qdot + dt*qddot;
    robot.updateState(q,qdot);
    
    t = (i-1)*dt;                                                           % Current time
    
    [pos,vel,acc] = trajectory.getState(t);                              	% Get the desired state
    
    
    switch method
        case 1 % PD+
            M = robot.getInertia();                                     	% Get inertia at current state
            Kp = 50*M;                                                   	% Position error gain
            Kd = 15*M;                                                    	% Velocity error gain
            tau = robot.pdPlus(vel,Kd,pos,Kp);                           	% Compute required joint torque
        case 2 % Inverse Dynamics
            Kp = 50;                                                        % Position error gain
            Kd = 15;                                                        % Velocity error gain
            tau = robot.invDynamics(acc,vel,Kd,pos,Kp);                     % Required joint torque
        otherwise
            error('Control method incorrectly specified.')
    end

    qddot = robot.getAcc(tau);                                              % This is just for simulation purposes
    
    if animate && mod(i,50) == 0
        robot.plot3D(q,'workspace',workspace);
    end
    
    % Save data for post-processing
    jointState(:,i,1) = q;
    jointState(:,i,2) = qdot;
    jointState(:,i,3) = tau;
    
    desiredState(:,i,1) = pos;
    desiredState(:,i,2) = vel;
end
temp = steps/toc;
disp(['Average loop frequency: ', num2str(temp), ' Hz.']);
%% Plot Performance Results
t = (0:steps-1)*dt;                                                         % Time vector for plots
for i = 1:3
    
    switch i
        case 1
            s = 180/pi;
            label = 'Deg';
        case 2
            s = 30/pi;
            label = 'RPM';
        case 3
            s = 1;
            label = 'Nm';
    end
    figure(i)
    set(gcf,'Color',[1 1 1])
    for j = 1:6
        subplot(3,2,j)
            plot(t,desiredState(j,:,i)*s,'k--','LineWidth',1);
            hold on
            plot(t,jointState(j,:,i)*s,'r','LineWidth',1);
            hold off
            box off
            title(['Joint ',num2str(j)])
            
            if j > 4
                xlabel('Time (s)')
            else
                set(gca,'XTick',[],'XColor','none')
            end
            
            if i == 1
                hold on
                scatter(times,qd(j,:)*s,'k','filled','SizeData',10)
                hold off
            end
            
            if mod(j,2) == 1
                ylabel(label)
            end
    end
end
