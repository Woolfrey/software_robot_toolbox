%% Pose
% Jonathan Woolfrey
% August 2019
%
% This class defines relative position and orientation between references
% frames. It is defined by a position vector (3x1) and a Rotation object.
%
% Poses can be concatenated by multiplying two pose objects together:
%
% Pose_3 = Pose_1*Pose_2
%
% Using Pose.matrix will return the 4x4 homogeneous transformation matrix
% associated with this pose representation.

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

classdef Pose < handle
    
    properties
        pos;                                                                % Position vector dim(3x1)
        rot;                                                                % Rotation object
    end
    
    methods (Access = public)
        
        %%%%%% Constructor %%%%%
        function obj = Pose(position,rotation)
            if nargin == 0
                obj.pos = zeros(3,1);
                obj.rot = Rotation('quat',1,[0;0;0]);
            else
                obj.pos = position;
                obj.rot = rotation;
            end
        end
                
        % Represent this pose as a homogeneous transformation matrix
        function ret = matrix(obj)
            ret = [obj.rot.matrix obj.pos; 0 0 0 1];                        % Return SE(3), or 4x4 matrix
        end
        
        % Return this inverse representation of this pose
        function ret = inverse(obj)                                         % Return inverse pose as an object
            R = obj.rot.matrix;
            ret = Pose(-R'*obj.pos, obj.rot.inverse);
        end
        
        % Compute the position and rotation error between this pose and
        % another
        function ret = error(obj,desired)
            ret = [desired.pos - obj.pos;                                	% Return the position error
                   obj.rot.error(desired.rot)];                            	% Return quaternion vector
        end
        
        % Transform a point to the reference frame represented by this pose
        function ret = transform(obj,point)
            ret = obj.pos + obj.rot.rotate(point);
        end
        
        % Produce a plot of this pose
        function plot(obj)
            obj.rot.plot(obj.pos);
        end
        
        %%%%% Operators %%%%%        
        function ret = mtimes(a,b)
            ret = Pose(a.transform(b.pos), a.rot*b.rot);           % Multiply 2 poses
        end
    end
    
end