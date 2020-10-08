%% Cartesian Trajectory
% Jonathan Woolfrey
% July 2020
%
% A Cartesian trajectory object. This takes several Pose objects and
% generates trajectories between them. To create the trajectory use:
%
% trajectory = Cartesian([Pose1 Pose2 ... Posen], [t1 t2 ... tn],"type"),
%
% where "type" is:
% - "trapezoidal"       for a trapezoidal velocity profile between 2 poses,
% - "quintic"           for a minimum-jerk trajectory between 2 poses
% - "cspline"           for anything more
%
% Calling the function trajectory.getState(time) returns:
% - Pose object
% - Velocity vector (6x1)
% - Acceleration vector (6x1)

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

classdef Cartesian < handle
    
    properties (Access = public)
        pose;                           % Array of pose objects
        time;                           % Array of times to reach each pose
    end
    
    properties (Access = private)
        position;                       % Position trajectory object
        orientation;                    % Orientation trajectory object
        type = 0;                     	% 1 = Quintic, 2 = Trapezoidal, 3 = Cubic Spline
    end
    
    methods (Access = public)
        
        %%%%% Constructor %%%%%
        function obj = Cartesian(pose, time, type)
            if length(pose) ~= length(time)
                error('Number of poses must equal number of time elements.');
            end
            
            obj.pose = pose;
            obj.time = time;
            
            if length(pose) == 2
                if strcmp(type,"trapezoidal")
                    obj.position = Trapezoidal(time(1),time(2),pose(1).pos,pose(2).pos);
                    obj.orientation = Trapezoidal(time(1),time(2),pose(1).rot.quat, pose(2).rot.quat);
                    obj.type = 2;
                elseif strcmp(type,"quintic")
                    obj.position = Quintic(time(1),time(2),pose(1).pos,pose(2).pos);
                    obj.orientation = Quintic(time(1),time(2),pose(1).rot.quat, pose(2).rot.quat);
                    obj.type = 1;
                else
                    error("Trajectory type must be 'trapezoidal' or 'quintic' between 2 poses.");
                end
            elseif length(pose) > 2
                if strcmp(type,"cspline")
                    temp = nan(3,length(pose));             % Temporary array of points
                    for i = 1:length(pose)
                        temp(:,i) = pose(i).pos;            % Add ith point to array
                    end
                    obj.position = CSpline(time,temp);      % Create cubic spline trajectory of points
                    temp = nan(4,length(pose));             % Temporary array of angle-axis
                    for i = 1:length(pose)
                        temp(1,i) = pose(i).rot.angle(); 	% Add ith angle to array
                        temp(2:4,i) = pose(i).rot.axis();   % Add the ith axis to array
                    end
                    obj.orientation = CSpline(time,temp);	% Create cubic spline of angle-axis
                    obj.type = 3;
                else
                    error("Trajectory type must be 'cspline' for more than 2 poses.");
                end
            else
                error("A minimum of 2 poses are required for a trajectory.");
            end
        end
        
        function [pos, vel, acc] = getState(obj,t)
            if obj.type == 1 || obj.type == 2               % Quintic Polynomial or Trapezoidal
                [p,pdot,pddot] = obj.position.lerp(t);      % Interpolate position
                [q,qdot,qddot] = obj.orientation.slerp(t);  % Interpolate orientation
                pos = Pose(p,Rotation('quat',q(1),q(2:4))); % Create Pose object
                vel = [pdot;qdot];                          % Combined velocity vector
                acc = [pddot;qddot];                        % Combined acceleration vector
            elseif obj.type == 3                            % Cubic Spline
                [p,pdot,pddot] = obj.position.getState(t);  % Interpolate position
                [q,qdot,qddot] = obj.orientation.getState(t); % Interpolate orientation as angle-axis

                axis = q(2:4)/norm(q(2:4));                 % Normalize the axis vector
                pos = Pose(p,Rotation('angleAxis',q(1),axis));  % Create Pose object
                vel = [pdot; qdot(2:4)];                    % Note that qdot(1) is the norm of the angular velocity vector
                acc = [pddot; qddot(2:4)];                  % Note that qddot(1) is the norm of the angular velocity vecor
            else
                error("Trajectory type not specified correctly.");
            end
        end
    end
    
    methods (Access = private)
    end
end