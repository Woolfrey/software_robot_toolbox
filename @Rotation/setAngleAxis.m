%% setAngleAxis
% Jonathan Woolfrey
%
% Set this rotation through angle-axis representation.
%
% When representing orientation is a quaternion (H), the elements of the
% quaternion are given by:
%
%               scalar = cos(0.5*angle)
%               vector = sin(0.5*angle)*axis
%
% Moreover, when represented as a 3x3 matrix R in SO(3), the axis of rotation
% is also the eigenvector such that:
%
%                   axis = R*axis,
%
% Inputs:
%   - angle (1x1)
%   - axis  (3x1)
%
% Outputs:
%   - none



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

function setAngleAxis(obj, angle, axis)
    if size(angle,1) ~= 1 || size(angle,2) ~= 1
        error('Expected a scalar value for the angle.');
    elseif size(axis,1) ~=3 || size(axis,2) ~= 1
        error('Expected a 3x1 vector for the axis.');
    else
        temp = [cos(0.5*angle); sin(0.5*angle)*axis];                       % Compute quaternion
        temp = temp/norm(temp);                                             % Normalize
        obj.quat = temp;                                                    % Assign
    end
end