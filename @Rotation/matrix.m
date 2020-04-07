%% Rotation Matrix
% Jonathan Woolfrey
%
% This function returns a 3x3 matrix representing the rotation in SO(3).
% By default, the Rotation object operates using quaternions (H).
%
% The Special Orthogonal group SO has the following properties:
%
%   - det(R) = 1
%   - R*R' = I (identity)
%   - norm(R) = 1
%
% Inputs:
%   - None
%
% Outputs:
%   - Rotation matrix (3x3)


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

function ret = matrix(obj)
    ret = zeros(3,3);
    ret(1,1) = obj.quat(1)^2 + obj.quat(2)^2 - obj.quat(3)^2 - obj.quat(4)^2;
    ret(1,2) = 2*(obj.quat(2)*obj.quat(3) - obj.quat(1)*obj.quat(4));
    ret(1,3) = 2*(obj.quat(2)*obj.quat(4) + obj.quat(1)*obj.quat(3));
    ret(2,1) = 2*(obj.quat(2)*obj.quat(3) + obj.quat(1)*obj.quat(4));
    ret(2,2) = obj.quat(1)^2 - obj.quat(2)^2 + obj.quat(3)^2 - obj.quat(4)^2;
    ret(2,3) = 2*(obj.quat(3)*obj.quat(4) - obj.quat(1)*obj.quat(2));
    ret(3,1) = 2*(obj.quat(2)*obj.quat(4) - obj.quat(1)*obj.quat(3));
    ret(3,2) = 2*(obj.quat(3)*obj.quat(4) + obj.quat(1)*obj.quat(2));
    ret(3,3) = obj.quat(1)^2 - obj.quat(2)^2 - obj.quat(3)^2 + obj.quat(4)^2;
end