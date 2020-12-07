%% Test Quaternion Spline
% Jon Woolfrey
% December 2020
%
% This script is used to test quaternion spline generation for orientation
% trajectories across multiple waypoints.

close all
clear all
clc

%% Establish Waypoints
Q1 = Rotation('rpy',[0      ;0          ;0]);
Q2 = Rotation('rpy',[pi/2   ;0          ;0]);
Q3 = Rotation('rpy',[pi/4   ;0          ;pi/4]);
Q4 = Rotation('rpy',[0	    ;pi/2      ;0]);
Q5 = Rotation('rpy',rand(3,1));
Q = [Q1,Q2,Q3,Q4,Q5];                               % Put in array
points = nan(3,length(Q));                          % Array of points to be used by trajectory object
for i = 1:length(Q)-1
    dQ = Q(i).inverse*Q(i+1);                       % Compute difference between two quaternions
    points(:,i) = dQ.angle*dQ.axis;                 % Assign point as product of angle and axis
end
points(:,end) = rand(1,3);                          % Final waypoint should be arbitrary
times = [0,2,5,6,8];                                % Time to reach each orientation
trajectory = CSpline(times,points,'quaternion');    % Create cubic spline trajectory using angle-axis waypoints

dt = 1/50;                                          % Discrete time step    
steps = times(end)/dt;                              % Number of steps in simulation

%%
state = nan(4,steps,3);                             % For recording data on position, velocity, acceleration
magnitude = nan(1,steps);                           % For checking unit norm of quaternion

for i = 1:steps
    t = (i-1)*dt;                                   % Current simulation time   
    [temp, omega, alpha] = trajectory.getState(t);  % Get current state
    angle = norm(temp);                             % Angle is the norm of this vector
    axis = temp/angle;                              % Axis of rotation is the unit vector
    dQ = Rotation('angleAxis',angle,axis);          % Rotation in the direction of the next waypoint
    
    % Determine the current spline
    if t >= times(end)                              % Final waypoint
        R = Q(end);
    elseif t <= times(1)                            % First waypoint
        R = Q(1);
    else
        for j = 1:length(Q)-1
            k = length(Q)-j;
            if t > times(k)                         % We are somewhere along the kth spline
                break
            end
        end
        R = Q(k)*dQ;                                % Increment rotation from the kth waypoint
    end
    state(:,i,1) = R.quat;                          % Save orientation
    state(1:3,i,2) = omega;                         % Save angular velocity
    state(1:3,i,3) = alpha;                         % Save angular acceleration
    magnitude = norm(R.quat);
end
%% Plot the Results
t = (0:steps-1)*dt;                                 % Create time array
subplot(3,1,1)
    plot(t,state(:,:,1),'LineWidth',1);             % Plot the quaternions over time
    hold on
%         plot(t,magnitude,'k*')                    % Check to see if unit norm
        waypoints = [Q(1).quat';Q(2).quat';Q(3).quat'; Q(4).quat';Q(5).quat']';
        for i = 1:3
            scatter(times,waypoints(i,:),'filled')  % Plot the waypoints
        end
    hold off
    box off
    ylabel('Quaternion')
subplot(3,1,2)
    plot(t,state(1:3,:,2),'LineWidth',1);       	% Plot the angular velocities
    ylabel('Velocity (rad/s)')
    box off
subplot(3,1,3)
    plot(t,state(1:3,:,3),'LineWidth',1);        	% Plot the angular acceleration
    ylabel('Acceleration (rad/s/s)')
    box off
    xlabel('Time (s)')
set(gcf,'Color',[1 1 1]);