%% SerialLink.getPartialJacobian(j,q,baseTF)
% Jonathan Woolfrey
% October 2020
%
% This function returns the partial derivative of the Jacobian matrix with
% respect to a particular joint on the manipulator.
%
% Inputs:
% - j (1x1):                The number of the joint to take the derivative of
% - q (nx1):                Optional joint configuration (default uses current manipulator state will be used)
% - baseTF (Pose object):   Optional base transformation (default uses current base pose)
%
% Output:
% - dJ/dq_j (6xn):          Exact numerical solution for the partial derivative of the Jacobian
%
% Verify the result with the following: Jdot = sum((dJ/dq_j)*qdot_j)

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

function ret = getPartialJacobian(obj,j,q,baseTF)
    ret = zeros(6,obj.n);               % Pre-allocate memory

    if nargin == 2                    	% Use current state
        v = obj.a;                     	% Axis of actuation for each joint
        d = obj.r;                    	% Distance from joint to end-effector
    else
        if nargin == 3
            baseTF = obj.base;          % Use current base pose
        end
        [~,FK] = obj.fk(q,baseTF);      % Compute forward kinematics
        v = obj.getAxis(FK);           	% Axis of actuation for each joint
        d = obj.getDist(FK);           	% Distance from joint to end-effector
    end

    for i = 1:obj.n
            if obj.link(j).isrevolute                                       % Revolute
                if obj.link(i).isrevolute                                   % Also revolute
                    if j < i
                        ret(:,i) = [cross(v(:,j),cross(v(:,i),d(:,i)))
                                           cross(v(:,j),v(:,i))];
                    elseif j == i
                        ret(1:3,i) = cross(v(:,j),cross(v(:,i),d(:,i)));
                    else
                        ret(1:3,i) = cross(v(:,i),cross(v(:,j),d(:,j)));
                    end
                elseif ~obj.link(j).isrevolute                              % Prismatic joint
                    if j < i
                        ret(1:3,i) = cross(v(:,j),v(:,i));
                    end
                end
            else % Prismatic
                if obj.link(i).isrevolute && j > i
                        ret(1:3,i) = cross(v(:,i),v(:,j));
                end
            end
    end
end