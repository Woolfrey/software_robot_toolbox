%% Trajectory Comparison
% Jon Woolfrey
%
% This script demonstrates a comparison in joint performance using a
% Trapezoidal trajectory versus Quintic Trajectory


% Copyright (C) Jon Woolfrey, 2019-2021
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
animate = 0;
dt = 1/robot.hertz;                   	% Discrete time step, for simulation purposes
Kp = 50;                                % Proportional feedback
Kd = 15;                                % Derivative feedback
method = 2;                             % 1 = Velocity control, 2 = Torque control
steps = 5/dt;                           % Total no. of steps to run the simulation
workspace = [-0.2 0.8 -0.5 0.5 0 1];    % Workspace for plotting the robot

%% Generate End-Effector Poses

% Set waypoint positions
p = nan(3,3);
p(:,1) = [0.3; 0.2; 0.1];
p(:,2) = [0.3; -0.1; 0.4];

% Set waypoint orientations as rotation objects
R(1,2) = Rotation();
R(1) = Rotation('rpy',[-pi, 0, pi/2]);
R(2) = Rotation('rpy',[pi/2, 0, -pi/2]);

% Create Pose object for each waypoint
P(1,2) = Pose();
for i = 1:2
    P(i) = Pose(p(:,i),R(i));
end

%% Solve Inverse Kinematics
q1 = [0.2712   -0.9903    1.9421   -2.5226   -1.5708    0.2712]';                           % First waypoint
q2 = [-0.4435    0.1934   -1.5777   -1.7571   -2.6980    1.5709]';                        % Second waypoint

%% Create trajectory object
t1 = 0;
t2 = 5;% Time to reach each waypoint
trajectory = Trapezoidal(t1,t2,q1,q2);                                             % Create cubic spline
% trajectory = Quintic(t1,t2,q1,q2);
%% Set up initial values & arrays
q = q1;                                                                % Initial joint configuration
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
    
    [pos,vel,acc] = trajectory.lerp(t);                              	% Get the desired state
      
    switch method
        case 1 % Velocity-level control                                                  	% Velocity error gain
            vd = vel + Kp*(pos - q);
            ad = 90*(vd - qdot);
        case 2 % Torque-level control
            ad = acc + Kp*(pos - q) + Kd*(vel - qdot);
        otherwise
            error('Control method incorrectly specified.')
    end
    tau = robot.invDynamics(ad);
    qddot = robot.getAcc(tau);                                              % This is just for simulation purposes
    
    if animate && mod(i,2) == 0
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
%             plot(t,desiredState(j,:,i)*s,'k--','LineWidth',1);
            hold on
            plot(t,jointState(j,:,i)*s,'k','LineWidth',1);
            hold off
            box off
            title(['Joint ',num2str(j)])
            
            if j > 4
                xlabel('Time (s)')
            else
                set(gca,'XTick',[],'XColor','none')
            end
            
%             if i == 1
%                 hold on
%                 scatter(times,qd(j,:)*s,'k','filled','SizeData',10)
%                 hold off
%             end
            
            if mod(j,2) == 1
                ylabel(label)
            end
    end
end
