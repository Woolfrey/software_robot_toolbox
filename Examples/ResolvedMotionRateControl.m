%% Resolved Motion Rate Control
% Jonathan Woolfrey
%
% Get a model of the Sawyer robot to track a Cartesian trajectory between 2
% poses. This method solves the end-effector control at the velocity level.
% PD+ control is required to execute the fundamental motor torque control.


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

load sawyer.mat;                                                            % Load the Sawyer model

%% General Simulation Parameters
animate = 1;                        % Turn animation on or off                                                             
dt = 1/100;                         % Discrete time step, for simulation purposes
Kp = 80;                            % Proportional gain
Kd = ceil(2*sqrt(Kp));          	% Derivative gain
steps = 10/dt;                      % No. of steps to run the simulation
workspace = [-0.2 0.8 -0.5 0.5 0 1];

%% Generate Initial and Final Pose
p1 = [0.6;-0.3; 0.7];                                                       % First position
p2 = [0.6; 0.3; 0.7];                                                     	% Second position
R1 = Rotation('rpy',[0; pi/2; 0]);                                          % First orientation
R2 = Rotation('rpy',[0; pi/2; 0]);                                          % Second orientation
T1 = Pose(p1,R1);                                                           % Create a pose object for the first
T2 = Pose(p2,R2);
%% Generate Trajectory Objects
t0 = 1;             % Start time                                                                % Start time
tf = 10;            % End time
trajectory = Cartesian([T1, T2], [t0, tf], 'trapezoidal');

%% Solve Inverse Kinematics for Initial Pose
% q0 = [-0.3132   -0.4681    1.0829   -1.1893   -0.2388    1.6943    0.5948]';
q0 = robot.ik(T1,0.2*randn(7,1),'method','inverse');                        % Solve the inverse kinematics
% robot.plot(q0,'workspace',workspace);                                  	% Plot the robot model to check the solution
disp('Proceed?')
pause;

%% Set Up Arrays
q = q0;                                                                     % Joint positions
qdot = zeros(7,1);                                                          % Joint velocities
qddot = zeros(7,1);                                                         % Joint accelerations
qdr = zeros(7,1);
jointState = nan(7,steps,3);                                                % For plotting data
trackingError = nan(2,steps);                                               % For plotting
m = nan(steps,1);                                                           % Manipulability
%% Run Simulation

tic;                                                                        % Start timer                                                           
for i = 1:steps
    % Update the current system state
    q = q + dt*qdot + 0.5*dt*dt*qddot;                                      % Update joint positions
    qdot = qdot + dt*qddot;                                                 % Update joint velocities
    robot.updateState(q,qdot);                                              % Update the robot state    
    t = (i-1)*dt;                                                           % Current simulation time
    
    % Get require end-effector trajectory at current time
    [pos,vel,acc] = trajectory.getState(t);
    
    % Compute joint control
    qdr = robot.maxManipulability(2);   % Redundant task
	qd = robot.rmrc(vel,pos,Kp,qdr);    % Joint velocity control
	qdd = 90*(qd - qdot);               % Joint acceleration control
    tau = robot.invDynamics(qdd);       % Torque control
    qddot = robot.getAcc(tau);          % This is only needed for simulation

    if animate && mod(i,25) == 0
        robot.plot3D(robot.q,'workspace',workspace);                        % Generate a plot of the robot
    end
    
    % Save data for plotting
    jointState(:,i,1) = robot.q;                                            % Joint positions
    jointState(:,i,2) = robot.qdot;                                         % Joint velocities
    jointState(:,i,3) = tau;                                                % Joint torques
    trackingError(1,i) = norm(pos.pos - robot.tool.pos)*1000;               % Position error
    Re = pos.rot*robot.tool.rot.inverse;    
    trackingError(2,i) = rad2deg(Re.angle);                                 % Orientation error
    m(i) = robot.manipulability;                                                % Manipulability
  
end
temp = steps/toc;                                                           % Stop timer and convert to frequency
disp(['Average loop frequency: ', num2str(temp), ' Hz.']);
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

figure(5)
plot(t,m,'k','LineWidth',1)
title('Manipulability')
xlabel('Time (s)')
box off
set(gcf,'Color',[1 1 1])

