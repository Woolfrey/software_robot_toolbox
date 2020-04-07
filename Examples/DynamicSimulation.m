%% Tests the Dynamics of the RobotToolbox
% Jonathan Woolfrey
% October 2019
%
% I created this script to visually inspect if my dynamics calculations
% were working correctly. It shows a Sawyer robot arm subject to
% gravitational acceleration. Note that the Sawyer dynamics were derived
% from the URDF parameters by Rethink Robotics, hence the centre of mass
% and inertia will not match up with the DH parameter frame of reference.


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

load sawyer.mat;

%%
robot.hertz = 100;                          % Control frequenc for the robot
dt = 1/robot.hertz;                         % Time-step
T = 60;                                     % Total time to run simulation
steps = T/dt;                               % No. of steps to loop through

q = randn(7,1);                             % Randomize the initial joint configuration
qdot = zeros(7,1);                          % Start at zero velocity
qddot = zeros(7,1);                         % Start at zero acceleration

jointState = nan(7,steps,3);                % Array for saving data

robot.base = Pose([0;0;0.5],Rotation('rpy',[0,pi/2,0])); % Set the base pose for the robot

for i = 1:steps
    q = q + dt*qdot + 0.5*dt*dt*qddot;      % Update joint position
    qdot = dt*qddot;                        % Update joint velocities
    
    robot.updateState(q,qdot);              % New robot state
    qddot = robot.getAcc(zeros(7,1));       % Figure out the joint accelerations
    
    if mod(i,100) == 0                      % Plot the robot at every 100th step
        robot.plot3D;
    end
    
    % Save data for analysis
    jointState(:,i,1) = q;
    jointState(:,i,2) = qdot;
    jointState(:,i,3) = qddot;
    
end

%% Plot the Results
t = (0:steps-1)*dt;
figure(1)
for i = 1:7
    subplot(4,2,i)
    plot(t,jointState(i,:,1)*180/pi,'k','LineWidth',1);
end

figure(2)
for i = 1:7
    subplot(4,2,i)
    plot(t,jointState(i,:,2)*30/pi,'k','LineWidth',1);
end

figure(3)
for i = 1:7
    subplot(4,2,i)
    plot(t,jointState(i,:,3),'k','LineWidth',1);
end


    