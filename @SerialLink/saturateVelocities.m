%% SerialLink.saturateVelocities()
% Jonathan Woolfrey
%
% This function saturates joint velocities so that they do not exceed their
% speed limits, or their position limits.
%
% Input:
% - vel             Current joint velocities (nx1)
% - pos             Current joint positions (nx1)
%
% Output:
% - Modified joint velocities (nx1)

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

function ret = saturateVelocities(obj,vel,pos)

    if nargin < 3
        pos = obj.q;                                % Use current joint positions
    end
    
    next = pos + vel/obj.hertz;                     % Compute next anticipated joint position vector

    % Slow down any joint that will exceed joint limits
    for i = 1:obj.n
        if next(i) > obj.link(i).qlim(2)
            vel(i) = 0.9*(obj.link(i).qlim(2) - pos(i))*obj.hertz;
        elseif next(i) < obj.link(i).qlim(1)
            vel(i) = 0.9*(obj.link(i).qlim(1) - pos(i))*obj.hertz;
        end
    end
    
    ret = vel;
end