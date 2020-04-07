%% MobileManipulator.rmrc()
% Jon Woolfrey
%
%
% Accounts for the base motion effects on the end-effector veloctiy.
% This is a work in progress.


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

function ret = rmrc(obj,vel,pos,Kp,Ki,qdr)
    r = obj.arm.tool.pos - obj.pose.pos;                                    % Distance from origin to end-effector
    xdot = vel - obj.vel;                                                   % Subtract base motion
    xdot(1:3) = xdot(1:3) - cross(obj.vel(4:6),r);                          % Angular velocity effects
    if nargin < 6 || obj.arm.n < 7
        qdr = zeros(7,1);                                                   % No redundant velocities
    end
    if nargin < 5
        Ki = 0;                                                             % No integral term
    end
    if nargin < 3
        pos = Pose;                                                         % Desired pose is arbitrary
        Kp = 0;                                                             % No position error feedback
    end
    ret = obj.arm.rmrc(xdot,pos,Kp,Ki,qdr);                               	% Compute rmrc for manipulator
end
