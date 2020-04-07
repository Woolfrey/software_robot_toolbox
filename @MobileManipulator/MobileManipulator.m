%% Mobile Manipulator
% Jonathan Woolfrey
% September 2019
%
% This class is a work-in-progress. Updates will come later.


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

classdef MobileManipulator < handle
    
    %%%%%%%%%% PROPERTIES %%%%%%%%%%
    properties (Access = public)
        acc = zeros(6,1);                                                   % Base acceleration
        arm = SerialLink([Link,Link,Link]);                              	% Serial link manipulator object
        baseTF = Pose();                                                    % Transform from base frame to manipulator origin
        inertia = eye(3);                                                   % Inertia
        mass = 50;                                                          % Mass
        pose = Pose();                                                      % Base pose
        vel = zeros(6,1);                                                   % Linear and angular velocity of the base
        torque;                                                             % Joint torques from base motion
        force;
        
        Mmb;                                                                % Maps base acceleration to manipulator
        Mbb;                                                                % Added inertia on base from manipulator
        Cmb;                                                                % Maps base velocity to manipulator
        Cbm;                                                                % Maps joint velocities to base
    end
    
    properties (Access = private)
        rdot;                                                               % Time-derivative of base to end-effector
        cdot;                                                               % Time-derivative of base to c.o.m.
    end
    
    %%%%%%%%%% METHODS %%%%%%%%%%%%s
    methods (Access = public)
        
        %%% Constructtor %%%
        function obj = MobileManipulator(varargin)
            % Process all options
            for i = 1:length(varargin)
                if ischar(varargin{i})
                    input = varargin{i};
                    switch input
                        case 'arm'
                            obj.arm = varargin{i+1};
                        case 'baseTF'
                            obj.baseTF = varargin{i+1};
                        case 'inertia'
                            obj.inertia = varargin{i+1};
                        case 'mass'
                            obj.mass = varargin{i+1};
                        case 'pose'
                            obj.pose = varargin{i+1};
                        otherwise
                            error('Incorrect option for MobileManipulator class.')
                    end
                end    
            end
        end
        
        %%% Need to put forward declarations of functions here. %%%
    end
    
    methods (Access = private)
        
        % Get the time-derivative of the distance to the end-effector
        function ret = getRdot(obj)            
            if nargin == 1                                                  % Use current state
                Jv = obj.arm.J(1:3,:);                                      % Velocity component for Jacobian
                qdot = obj.arm.qdot;
                r = obj.arm.tool.pos;                                       % Distance to end-effector
                omega = obj.vel(4:6);                                       % Angular velocity of base
            else                                                            % Otherwise, compute necessary information
               % Need to fill this in 
            end
            ret = Jv*qdot + cross(omega,r);
        end
        
        % Get the time-derivative of the distance to each link c.o.m.
        function ret = getCdot(obj)
            ret = zeros(3,obj.arm.n);                                       % Pre-allocate memory
            if nargin == 1                                                  % Use current state information
                [com,Jm] = obj.arm.getMassGeometry();                       % Centre of mass for each link & Jacobian
                qdot = obj.arm.qdot;                                        % Joint velocities for the manipulator
                omega = obj.vel(4:6);                                       % Angular velocity of the base
            else
                % Need to fill this in later
            end
            for i = 1:obj.arm.n
                ret(:,i) = Jm(1:3,:,i)*qdot + cross(omega,com(:,i));
            end
        end
        
    end
 
end