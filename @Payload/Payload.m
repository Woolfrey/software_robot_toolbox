%% Payload Class
% Jon Woolfrey
% October 2020
%
% This class defines a physical payload on the end of a robot manipulator.
%
% TO DO:
%   - Currently assumes all manipulator joints are revolute. Need to
%     account for prismatic joints!

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
        TF = Pose([0;0;0.1],Rotation('rpy',[pi,0,pi/2])); % Transform from end-effector to centre of mass of payload
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
        function obj = Payload(mass,inertia,pose)
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
                    obj.pose = pose;
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