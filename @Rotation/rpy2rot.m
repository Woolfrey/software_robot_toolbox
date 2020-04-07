%% rpy2rot
% Jonathan Woolfrey
%
% This script converts roll, pitch, and yaw angles to an equivalent
% rotation matrix in SO(3).
%
% Inputs:
%   - Roll, pitch, and yaw angles as 3 individual elements, OR
%   - A 3x1 vector of the roll, pitch, and yaw
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

function ret = rpy2rot(r,p,y)
    if nargin == 1
        if length(r) ~= 3
            error('Expected a 3x1 vector.');
        else
            ret = Rotation.rotx(r(1))*Rotation.roty(r(2))*Rotation.rotz(r(3));
        end
    elseif nargin == 3
        ret = Rotation.rotx(r)*Rotation.roty(p)*Rotation.rotz(y);
    else
        error('Expected a 3x1 vector.');
    end
end