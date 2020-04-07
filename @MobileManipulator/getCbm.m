%% MobileManipulator.getCbm
% Jon Woolfrey
%
% Get the Coriolis/centripetal matrix of forces induced on the base from
% the manipulator joint velocities.
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

% Maps manipulator velocities to forces on the base
function ret = getCbm(obj)
    ret = zeros(6,obj.arm.n);                                       % Pre-allocate memory
    if nargin == 1                                                  % Use current state information
        [d,Jm,I,Jmdot,Idot] = obj.arm.getMassGeometry();
    else
        %%% Need to fill this in %%%
    end
    for j = obj.arm.n                                               % Cycle through every link
        ret(1:3,:) = ret(1:3,:) + obj.arm.link(j).mass*Jmdot(1:3,:,j);
        ret(4:6,:) = ret(4:6,:) + Jm(4:6,:,j)*Idot(:,:,j) ...
                                + Jmdot(4:6,:,j)*I(:,:,j) ...
                                - obj.arm.link(j).mass*skew(d(:,j))*Jmdot(1:3,:,j);
    end
end