%% SerialLink.getJacobian()
% Jonathan Woolfrey
%
% This function computes the Jacobian matrix for a serial link manipulator
%
% Inputs:
%   - q:        Joint positions (nx1)
%   - baseTF:   Base pose w.r.t. inertial frame (Pose object)
%
% Outputs:
%   - J:        Jacobian matrix (6xn)
%
% If no input is given, this function will use the current manipulator
% state from when updateState(q,qdot,baseTF) was last called.



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

function ret = getJacobian(obj,q,baseTF)
    ret = zeros(6,obj.n);                                                   % Pre-allocate memory
    if nargin == 1                                                          % No inputs, use current state
        axis = obj.a;                                                       % Axis of actuation for each joint
        dist = obj.r;                                                       % Distance from joint to end-effector
    else                                                                    % Arguments given
        if nargin == 2
            baseTF = obj.base;
        end
        [~,FK] = obj.fk(q,baseTF);                                          % Compute forward kinematics
        axis = obj.getAxis(FK);                                             % Get axis of actuation for each joint
        dist = obj.getDist(FK);                                             % Get distance from joint to tool
    end
    for i = 1:obj.n
        if obj.link(i).isrevolute                                           % Joint is revolute
            ret(:,i) = [cross(axis(:,i),dist(:,i))
                                axis(:,i)];
        elseif ~obj.link(i).isrevolute                                      % Joint is prismatic
            ret(1:3,i) = axis(:,i);
        else
            error("Link type incorrectly specified. Cannot compute Jacobian matrix.");
        end
    end
end