%% Forward Kinematics
% Jonathan Woolfrey
%
% This script computes the forward kinematics chain for a serial link
% manipulator.
%
% Inputs:
%   - q: Joint positions (nx1)
%   - baseTF: Base pose relative to inertial frame (Pose object)
%
% Outputs:
%   - FK: The end-effector pose of the manipulator (Pose object)
%   - ALL: Array of all link poses
%
% If no arguments are given, this function will use the current manipulator
% state. For example:
%       robot.updateState(q,qdot,baseTF);
%       EE = robot.fk();


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

function [FK, ALL] = fk(obj,q,baseTF)
    
    switch nargin
        case 1
            q = obj.q;
            baseTF = obj.base;
        case 2
            baseTF = obj.base;
        case 3
            % No need to do anything
    end
    
    ALL(obj.n,1) = Pose();                              	% Create array of poses
	ALL(1) = baseTF*obj.link(1).getPose(q(1));            	% Multiply first link pose by base
    for i = 2:obj.n
        ALL(i) = ALL(i-1)*obj.link(i).getPose(q(i));      	% Compute FK chain
    end

    FK = ALL(obj.n);                                        % End-effector pose is last in chain
end
