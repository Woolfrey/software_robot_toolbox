%% MobileManipulator.getCmb
% Jon Woolfrey
%
% Get the Coriolis/centripetal matrix of forces on the manipulator joint
% from the base velocities.
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

% Maps base velocities to forces on the manipulator
function ret = getCmb(obj)
    ret = zeros(obj.arm.n,6);                                       % Pre-allocate memory
    if nargin == 1                                                  % Use current state information
        [~,Jm,~,~,Idot] = obj.arm.getMassGeometry();                
        ddot = obj.cdot;
    else
        %%% Need to fill this in %%%
    end
    for j = 1:obj.arm.n                                             % Cycle through every link
        ret(:,4:6) = ret(:,4:6) + Jm(4:6,:,j)'*Idot(:,:,j)...
                     - obj.arm.link(j).mass*Jm(1:3,:,j)'*skew(ddot(:,j));
    end
end