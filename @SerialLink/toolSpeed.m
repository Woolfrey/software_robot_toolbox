%% SerialLink.toolSpeed()
% Jonathan Woolfrey
%
% This function computes the tool (end-effector) speed of a serial link
% manipulator given the joint positions and velocities. The velocity
% relationship between the joint space and tool space is given by:
%
%           xdot = J(q)*qdot,
%
% where J(q) = df/dq (6xn) is the manipulator Jacobian.
%
% Inputs:
%   - q:        Joint positions (nx1)
%   - qdot:     Joint velocities (nx1)
%
% Output:
%   - xdot:     Tool velocity (6x1)



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

function ret = toolSpeed(obj,q,qdot)
    if nargin == 1
        qdot = obj.qdot;                                                    % Use current state
        J = obj.getJacobian();                                              % Compute Jacobian at current configuration
    else
        J = obj.getJacobian(q);                                             % Compute Jacobian for given pose                                       
    end
    
    ret = J*qdot;                                                           
end