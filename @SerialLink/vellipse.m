%% SerialLink.vellipse()
% Jon Woolfrey
% October 2020
%
% This function plots the velocity ellipsoid for a serial link manipulator.
% This represents the relative ease that the end-effector can move in
% Cartesian space. Since an ellipsoid cannot be visualized for dimensions
% higher than 3, only the velocity component is plotted. An optional joint
% configuration can be input as an argument, otherwise the current
% manipulator state is used.

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

function vellipse(obj,q)
    if nargin == 1
        q = obj.q;                      % Use current joint positions
        centre = obj.tool.pos;          % Use current end-effector pose
    else
        centre = obj.fk(q).pos;         % Compute end-effector position at given joint configuration
    end
    
    J = obj.getJacobian(q);
    A = inv(J(1:3,:)*J(1:3,:)');
    hold on
    plotEllipse(5*A,'Centre',centre);
    hold off
end