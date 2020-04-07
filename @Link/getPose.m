%% Link.getPose()
% Jonathan Woolfrey
%
% This function gets the local pose/transform of a link relative to a
% preceeding link for a given joint position.
%
% Input:
%   - q: Joint position (1x1)
%
% Output:
%   - Pose object



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

function ret = getPose(obj,q)                                               % Return the local link transform
    if obj.isrevolute
        obj.theta = q + obj.offset;                                         % z-rotation is joint angle
    else
        obj.d = q + obj.offset;                                             % z-translation is joint position
    end

    % Pre-computation of tigonometric functions to make things
    % a little easier
    sa = sin(obj.alpha);
    st = sin(obj.theta);
    ca = cos(obj.alpha);
    ct = cos(obj.theta);

    angle = acos(0.5*(ca + ct + ca*ct -1));                                 % acos(0.5*(trace(R)-1))

    if abs(angle) < 1E-3
        axis = [1;0;0];                                                     % Axis is arbtitrary
    else
        axis = (1/(2*sin(angle)))*[sa + sa*ct; sa*st; st + ca*st];          % Axis of rotation from SO(3)
    end
    p = [obj.a*ct; obj.a*st; obj.d];                                        % Position/translation vector
    ret = Pose(p,Rotation('angleAxis',angle,axis));
end