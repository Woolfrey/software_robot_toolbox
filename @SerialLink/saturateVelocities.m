%% SerialLink.saturateVelocities()
% Jonathan Woolfrey
%
% This function saturates joint velocities so that they do not exceed their
% speed limits, or their position limits.
%
% The default method is to attenuate all the joint velocities individually.
% Inputting the method as 'scaled' will attenuate all velocities equally so
% that the desired direction of the end-effector motion is maintained.



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

function ret = saturateVelocities(obj,pos,vel,method)
    if nargin == 3
        method = 'individual';
    end
    
    % First check if any joint velocities exceed limits
    if strcmp(method, 'scaled')                                             % Scale joints to maintain end-effector direction
        max = 0;                                                                    
        index = 1;
        for i = 1:obj.n
            if abs(vel(i)) > obj.link(i).vlim && abs(vel(i)) > max          % Joint velocity exceeds limit & is max observed
                max = abs(vel(i));                                          % Update maximum
                index = i;                                                  % Record joint number
            end
        end
        
        if max > 0                                                          % At least one joint has exceeded speed limit
            scalar = obj.link(index).vlim/max;                              % Compute scalar
            for i = 1:obj.n
                vel(i) = vel(i)*scalar;                                     % Scale each joint velocity equally
            end
        end
    else
        for i = 1:obj.n
            if abs(vel(i)) > obj.link(i).vlim
                vel(i) = sign(vel(i))*obj.link(i).vlim;                     % Scale each joint individually
            end
        end
    end
    
    % Then slow down any joints that are going to hit a joint limit
    next = pos + vel/obj.hertz;                                             % Compute next expected joint position
    for i = 1:obj.n
        if next(i) > obj.link(i).qlim(2)
            vel(i) = 0.9*(obj.link(i).qlim(2) - pos(i))*obj.hertz;
        elseif next(i) < obj.link(i).qlim(1)
            vel(i) = 0.9*(obj.link(i).qlim(1) - pos(i))*obj.hertz;
        end
    end
    
    ret = vel;
end