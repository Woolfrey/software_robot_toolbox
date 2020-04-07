%% setRPY
% Jonathan Woolfrey
%
% Set rotation via the roll, pitch, and yaw angle representation.
%
%
% Inputs:
%   - Roll, Pitch, and Yaw angles
%
% Outputs:
%   - None
%
% The input may be either specified as 3 individual scalars, each
% representing roll, pitch, and yaw respectively. Or, the input can be a
% 3-element vector. For example:
%
% R.setrpy(r,p,y);  % Set the rotation from 3 separate scalars,
%
% OR
%
% R.setrpy([r,p,y]); % Set the rotation using a 3-element vector



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

function setRPY(obj,r,p,y)
    if nargin == 2                                                          % Input is a 3x1 vector
        roll = r(1);
        pitch = r(2);
        yaw = r(3);
    elseif nargin == 4                                                      % Input is 3 separate scalars
        roll = r;
        pitch = p;
        yaw = y;
    end
    R = Rotation.rotx(roll)*Rotation.roty(pitch)*Rotation.rotz(yaw);                 % Calculate 3x3 rotation matrix
    obj.quat = Rotation.rot2quat(R);
end