%% Link Class
% Jonathan Woolfrey
% August 2019
%
% This class defines a simple link object. Creating this object without any
% input parameters results in a default link. The fundamental geometry of a
% link is defined by the DH parameters in the following order:
% - theta:      Rotation about the z-axis
% - d:          Translation along the z-axis
% - a:          Translation along the x-axis
% - alpha:      Rotation about the x-axis
%
% Setting theta as [] results in a revolute joint. Conversely, setting d as
% [] results in a prismatic joint.
%
% All link properties can be set on creation using:
%
% link = Link("property1",input1, "property2", input2, ... )

% The other properties, besides the DH parameters, are:
% - colors:     Used for rendering a 3D model
% - com:        The location of the centre of mass (m) from the origin
% - faces:      Used for rendering a 3D model
% - inertia:    The 3x3 inertia tensor (kg-m^2)
% - mass:       The mass of the link (kg)
% - qlim:       The upper and lower limits for the joint on this link
% - tlim:       The torque limit for the joint on this link
% - vertices:   Used for rendering a 3D model
% - vlim:       The velocity limit for the joint on this link
% - offset:     Offset from the "zero" position of the joint in the DH
% model from the real model.

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

classdef Link
    
    %%%%%%%%%  PROPERTIES %%%%%%%%%%
    properties (Access = public)
        a = 1;                                                              % DH Parameter
        alpha = 0;                                                          % DH Parameter
        colors = [];                                                        % Used for 3D modeling
        com = [-0.5;0;0];                                               	% Centre of mass in local link frame (m)
        d = 0;                                                              % DH Parameter
        faces = [];                                                         % Used for 3D modeling
        inertia = [0.10 0.00 0.00                                           
                   0.00 0.10 0.00
                   0.00 0.00 0.10];                                      	% Inertia matrix in local frame (kg.m^2)
        mass = 1;                                                          	% Mass of the link (kg)
        qlim = [-2*pi 2*pi];                                                % Joint limits
        theta = [];                                                         % DH Parameter
        tlim = 20;                                                          % Torque limits
        isrevolute = true;                                                  % revolute or prismatic
        vertices = [];                                                      % Used for 3D modeling
        vlim = 100*2*pi/60;                                                 % Velocity limits (in rad/s)
        offset = 0;                                                         % Joint offset to account for DH parameters
    end
    
    %%%%%%%%%  METHODS %%%%%%%%%%
    methods (Access = public)
            
            %%%%% Constructor %%%%%
            function obj = Link(varargin)
                if nargin == 0
%                     disp("A default link has been created.")
                else
                    for i = 1:nargin
                        if ischar(varargin{i})
                            switch varargin{i}
                                case 'a'
                                    obj.a = varargin{i+1};
                                case 'alpha'
                                    obj.alpha = varargin{i+1};
                                case 'd'
                                    obj.d = varargin{i+1};
                                case 'theta'
                                    obj.theta = varargin{i+1};
                                case 'offset'
                                    obj.offset = varargin{i+1};
                                case 'colors'
                                    obj.colors = varargin{i+1};
                                case 'com'
                                    obj.com = varargin{i+1};
                                case 'faces'
                                    obj.faces = varargin{i+1};
                                case 'inertia'
                                    obj.inertia = varargin{i+1};
                                case 'mass'
                                    obj.mass = varargin{i+1};
                                case 'qlim'
                                    obj.qlim = varargin{i+1};
                                case 'tlim'
                                    obj.tlim = varargin{i+1};
                                case 'vertices'
                                    obj.vertices = varargin{i+1};
                                case 'vlim'
                                    obj.vlim = varargin{i+1};
                                otherwise
                                    error(['Input ',varargin{i},' is not a valid option for the Link class.']);
                            end
                        end
                    end
                end

                if isempty(obj.theta)
                    obj.isrevolute = true;                                  % Revolute joint
                elseif isempty(obj.d)
                    obj.isrevolute = false;                                 % Prismatic joint
                else
                    error("Joint type incorrectly specified.");
                end
            end
            
            %%% Declaration of External Functions
            ret = getPose(obj,q)
    end 
end