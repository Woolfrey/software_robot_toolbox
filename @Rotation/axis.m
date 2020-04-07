%% Axis of Rotation
% Jonathan Woolfrey
%
% This function gets the axis of rotation represented by a Rotation object.
% If a rotation R is represented in SE(3), then the axis a is an
% eigenvector of R such that:
%
%                       a = R*a.
%
% For a rotation represented in quaternion space (H), then the values are
% given by:
%
%                   scalar = cos(0.5*angle)
%                   vector = sin(0.5*angle)*axis
%
%   Inputs:
%       - None.
%
%   Outputs:
%       - Axis of rotation (3x1) which is also unit norm.


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

function ret = axis(obj)
    angle = obj.angle();                                                    % Get the angle of rotation from this quaternion
    if angle < 1E-3 || pi - angle < 1E-3 || 2*pi - angle < 1E-3             % Check the magnitude of the angle
        ret = zeros(3,1);                                                   % Arbitrary
    else
        axis = obj.quat(2:4)/sin(0.5*angle);                                % Compute the axis of rotation
        ret = axis/norm(axis);                                              % Normalize
    end
end