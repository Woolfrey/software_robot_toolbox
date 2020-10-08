%% Circular Trajectory Class
% Jonathan Woolfrey
% August 2019
%
% This generates circular trajectories. It builds upon the quintic
% polynomial.

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

classdef Circular < handle
    
    %%%%%%%%%% PROPERTIES  %%%%%%%%%%
    properties (Access = public)
        angle;                                                              % Starting angle on the circle
        radius;                                                             % Radius of the circle
        revs;                                                               % No. of revolutions to run through
        trajectory;                                                         % Fundamental trajectory object
    end
    
    properties (Access = private)
    end
    
    
    %%%%%%%%%% METHODS  %%%%%%%%%%
    methods (Access = public)
        
        %%%%% Constructor %%%%%
        function obj = Circular(t0,tf,angle,revs,radius)
            obj.angle = angle;                                              % Assign start angle
            obj.radius = radius;                                            % Assign radius of circle
            obj.revs = revs;                                                % Assign number of revolutions
            obj.trajectory = Quintic(t0,tf,angle,angle+revs*2*pi);        	% Generate fundamental trajectory
        end
        
        function [pos,vel,acc] = getPoint(obj,t)                            % Get current point on circle
            
            [a,ad,add] = obj.trajectory.lerp(t);                            % Get angle and derivatives
            
            pos = [ obj.radius*cos(a)
                    obj.radius*sin(a)
                    0];
            
            vel = [ -obj.radius*sin(a)*ad
                     obj.radius*cos(a)*ad
                     0];
                 
            acc = [-obj.radius*(sin(a)*add + cos(a)*ad^2)
                    obj.radius*(cos(a)*add - sin(a)*ad^2)
                    0];            
        end
    end
    
    methods (Access = private)
    end
end