%% Inverse Rotation
% Jonathan Woolfrey
%
% This function returns the inverse of a Rotation object. When using
% quaternions (H) to represent rotations, the inverse is equivalent to the
% conjugate such that:
%
%       Q*Q' = [1, 0 0 0],
%
% where Q' is the conjugate of Q.
%
% Inputs:
%   - None
%
% Outputs:
%   - Rotation object


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

function ret = inverse(obj)
        ret = Rotation();                                               % Create rotation object
        ret.quat = [obj.quat(1);-obj.quat(2:4)];                        % Negate the vector component
end