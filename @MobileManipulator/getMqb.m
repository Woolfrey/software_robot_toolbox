%% MobileManipulator.getMmb
% Jon Woolfrey
%
% Get the inertia matrix of base acceleration in the manipulator.
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

% Maps base acceleration to forces on the manipulator and vice versa
function ret = getMmb(obj)
    ret = zeros(obj.arm.n,6);                                               % Pre-allocate memory
    if nargin == 1                                                          % Use current state
        [com, Jc, I] = obj.arm.getMassGeometry();                           % Inertia properties            
    else
        % Need to fill in this later
    end
    for i = 1:obj.arm.n                                                     % Cycle through every link
        ret = ret + [obj.arm.link(i).mass*Jc(1:3,:,i)', ...
               Jc(4:6,:,i)'*I(:,:,i) - obj.arm.link(i).mass*Jc(1:3,:,i)'*skew(com(:,i))];
    end
end