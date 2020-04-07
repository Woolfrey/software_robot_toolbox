%% Roll, Pitch, Yaw Angles
% Jonathan Woolfrey
%
% This function returns the roll, pitch, and yaw angles represented by a
% Rotation object. The rotation object formally uses quaternions (H) to
% compute rotations. While not very intuitive, they are computationally
% efficient.
%
% Euler angles are a more intuitive representation for interpreting
% rotations, but they suffer from gimbal lock.
%
% Inputs:
%   - None
%
% Outputs:
%   - roll, pitch, and yaw angles (3x1)
%
% WARNING: The results may be incorrect at gimbal lock configurations!



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

function ret = rpy(obj)
    R = obj.matrix();                                                       % Express rotation as SO(3)
    r = atan2(R(3,2),R(3,3));                                               % Roll
    y = atan2(R(2,1),R(1,1));                                               % Yaw
    if cos(y) == 0
        p = atan2(-R(3,1),R(2,1)/sin(y));                                   % Pitch
    else
        p = atan2(-R(3,1),R(1,1)/cos(y));                                   % Alternative calculating of pitch
    end
    ret = [r;p;y];                                                          % Return as 3x1 vector
end