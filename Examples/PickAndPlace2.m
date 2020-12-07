%% Pick And Place 2
% Jonathan Woolfrey
%
% Get a model of a robot to lift and move an object using the Payload
% class.

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

load kukaiiwa7.mat;                     % Robot model                                            % Load the Sawyer model
load itemBox.mat;                       % Payload model
robot.payload = itemBox;                % Assign itembox to SerialLink           
robot.payload.exists = true;            % Turn on dynamic calcs
%% General Simulation Parameters
animate = 1;                            % 1 = Animate                                                          
dt = 1/100;                             % Discrete time-step, inverse of frequency
Kp = 1000;                              % Proportional gain
Kd = ceil(2*sqrt(Kp));                  % Derivative gain
steps = 15/dt;                          % Total no. of steps to run simulation
workspace = [-0.2 0.8 -0.5 0.5 0 1];    % Used when animating

%% Generate Poses and Trajectory
P1 = Pose([0.4; 0.25; 0.2], Rotation('rpy',[-pi,0,0]));     % First
P2 = Pose([0.4; 0.25; 0.6], Rotation('rpy',[0,0,0]));       % Second
P3 = Pose([0.4;-0.25; 0.6], Rotation('rpy',[-pi,0,0]));     % Third
P4 = Pose([0.4;-0.25; 0.2], Rotation('rpy',[-pi,0,0]));     % Fourth
trajectory = Cartesian([P1,P2,P3,P4],[0,4,8,12],'cspline');
%% Solve Inverse Kinematics for Initial Pose
% q0 = robot.ik(P1,0.2*randn(7,1),'method','inverse');    	% Solve the inverse kinematics
% robot.plot(q0,'workspace',workspace);                    	% Plot the robot model to check the solution
q0 = [-0.0140    0.8132   -2.4062   -1.8796    2.3598    0.7640   -1.9775]';
disp('Proceed? (Press any key)')
pause;

%% Set Up Arrays
q = q0;                      	% Joint positions
qdot = zeros(7,1);           	% Joint velocities
qddot = zeros(7,1);             % Joint accelerations
jointState = nan(7,steps,3); 	% For plotting data
trackingError = nan(2,steps); 	% For plotting

%% Run Simulation

tic;                                                       	% Start timer                                                           
for i = 1:steps
    % Update the current system state
    q = q + dt*qdot + 0.5*dt*dt*qddot;                    	% Update joint positions
    qdot = qdot + dt*qddot;                                	% Update joint velocities
    robot.updateState(q,qdot);                             	% Update the robot state    
    t = (i-1)*dt;                                         	% Current simulation time
    
    % Get require end-effector trajectory at current time
    [pos,vel,acc] = trajectory.getState(t);
    
    % Compute joint control
    qddr = -robot.D*qdot;                                   % Redundant task
    tau = robot.rac(acc, vel, Kd, pos, Kp, qddr);           % Compute joint torques
    qddot = robot.getAcc(tau);                              % This is only needed for simulation
    
    if animate && mod(i,25) == 0
        robot.plot3D(robot.q,'workspace',workspace,'view',[80 30]);        % Generate a plot of the robot
        itemBox.plot();
    end
    
    % Save data for plotting
    jointState(:,i,1) = robot.q;                                            % Joint positions
    jointState(:,i,2) = robot.qdot;                                         % Joint velocities
    jointState(:,i,3) = tau;                                                % Joint torques
    trackingError(1,i) = norm(pos.pos - robot.tool.pos)*1000;               % Position error
    Re = pos.rot*robot.tool.rot.inverse;    
    trackingError(2,i) = rad2deg(Re.angle);                                 % Orientation error
end
temp = steps/toc;                                                           % Stop timer and convert to frequency
disp(['Average loop frequency: ', num2str(temp), ' Hz.']);
pause();
%% Plot Results

t = (0:steps-1)*dt;                                                         % Time vector
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
    set(gcf,'Color',[1 1 1]);
    
    for j = 1:7
        subplot(4,2,j)
        plot(t,jointState(j,:,i)*s,'k','LineWidth',1);
        title(['Joint ',num2str(j)])
        if mod(j,2) == 1
            ylabel(label);
        end
        if j < 6
            set(gca,'XTick',[],'XColor','none')
        else
            xlabel('Time (s)')
        end
        box off
    end
    
end

figure(4)
    subplot(2,1,1)
        plot(t,trackingError(1,:),'k','LineWidth',1)
        box off
        ylabel('Position Error (mm)')
        set(gca,'XTick',[])
    subplot(2,1,2)
        plot(t,trackingError(2,:),'k','LineWidth',1)
        box off
        ylabel('Orientation Error (deg)')
        xlabel('Time (s)')
set(gcf,'Color',[1 1 1])
