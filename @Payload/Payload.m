%% Payload Class
% Jon Woolfrey
% October 2020
%
% This class defines a physical payload on the end of a robot manipulator.
%
% Constructor arguments:
% - mass (1x1)              The mass of the object (kg)
% - inertia (3x3)           Inertia tensor matrix (kg.m^2)
% - TF (Pose object)        The pose of the centre of mass relative to the end-effector of the manipulator
%
% By setting payload.exists = true, the SerialLink class will automatically
% compute the joint torques needed to hold/move the object.
%
% Specify colors, faces, and vertices to plot a 3D model of the payload
% with payload.plot().
%
% TO DO:
%   - Currently assumes all manipulator joints are revolute. Need to
%     account for prismatic joints!

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

classdef Payload < handle
    
    %%%%%%%%% PROPERTIES %%%%%%%%%%
    properties (Access = public)
        colors = [];                    % For 3D modeling
        exists = false;                 % Used to turn on/off dynamic calculations
        faces = [];                     % For 3D modeling
        inertia = zeros(3,3);           % Inertia tensor in local frame
        mass = 0;                       % Mass of the object
        name = "payload";               % Used for identifying the object
        pose = Pose();                  % Current pose relative to world
        TF = Pose([0;0;0],Rotation('rpy',[0 0 0])); % Transform from end-effector to centre of mass of payload
        vertices = [];                  % For 3D modeling
    end

    properties (Access = private)
        I = zeros(3,3);                 % Inertia in global frame
        Idot = zeros(3,3);            	% Time-derivative of inertia in global frame
        J = zeros(6,6);                 % Jacobian matrix
        Jdot = zeros(6,6);              % Time-derivative of Jacobian
        r = zeros(3,6);               	% Translation from each joint to centre of mass
        rdot = zeros(3,6);             	% Time-derivative of translation
    end

    %%%%%%%%%% METHODS %%%%%%%%%%
    methods (Access = public)
        %%%%% Constructor %%%%%
        function obj = Payload(mass,inertia,TF)
            switch nargin
                case 1
                    error("Require both mass and inertia tensor to define a payload object.")
                case 2
                    if size(inertia,1) ~= 3 || size(inertia,2) ~= 3
                        error("Inertia must be a 3x3 matrix.")
                    end
                    obj.mass = mass;
                    obj.inertia = inertia;
                    obj.exists = true;
                case 3
                    obj.mass = mass;
                    obj.inertia = inertia;
                    obj.TF = TF;
                    obj.exists = true;
            end
        end
        
        %%% Forward declarations %%%
        ret = getCoriolis(obj);
        ret = getGrav(obj,g);
        ret = getInertia(obj);
    end

    methods (Access = private)

    end
end