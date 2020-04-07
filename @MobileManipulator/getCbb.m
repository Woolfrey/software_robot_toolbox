%% MobileManipulator.getCbb()
% Jon Woolfrey
%
% Get the Coriolis/centripetal matrix of forces on the base from the base
% velocities moving the manipulator.
% This is a work-in-progress.

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


% Maps manipulator velocities to forces on the base
function ret = getCbb(obj)
    ret = zeros(6,6);                                                       % Pre-allocate memory
    if nargin == 1                                                          % Use current state
        [d,~,~,~,Idot] = obj.arm.getMassGeoemetry();
        ddot = obj.cdot;
    else
        %%% Need to fill this in %%%
    end
    for i = 1:obj.arm.n
        ret(1:3,4:6) = ret(1:3,4:6) - obj.arm.link(i).mass*skew(ddot(:,i));
        ret(4:6,4:6) = ret(4:6,4:6) + Idot(:,:,i)...
                       + obj.arm.link(i).mass*skew(d(:,i))*skew(ddot(:,i));
    end
end