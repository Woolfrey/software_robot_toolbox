%% MobileManipulator.getMbb()
% Jon Woolfrey
%
% Get the inertia matrix of the manipulator with respect to the base.
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

% Maps the manipulator inertia to the base
function ret = getMbb(obj)
    ret = zeros(6,6);                                                       % Pre-allocate memory
    if nargin == 1                                                          % Use current state
        [com,~,I] = obj.arm.getMassGeometry;    
    else
        %%% Need to fill this in %%%
    end
    for j = 1:obj.arm.n                                                     % Cycle through every link
        m = obj.arm.link(j).mass;
        S = skew(com(:,j));
        ret = ret + [m*eye(3), -m*S
                     m*S     ,  I(:,:,j) - m*S];
    end
end