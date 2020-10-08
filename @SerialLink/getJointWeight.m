%% SerialLink.getJointWeight()
% Jonathan Woolfrey
%
% This function returns a weighting matrix for joint limit avoidance of a
% redundant manipulator. This is based on the following paper:
% Tan Fung Chan and R. V. Dubey, "A weighted least-norm solution based scheme
% for avoiding joint limits for redundant joint manipulators," in IEEE 
% Transactions on Robotics and Automation, vol. 11, no. 2, pp. 286-292, April 
% 1995, doi: 10.1109/70.370511.

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

function ret = getJointWeight(obj,q,qdot)
    
    ret = eye(obj.n);                         % Joint weighting matrix to return
    for i = 1:obj.n
        u = obj.link(i).qlim(2) - q(i);       % Proximity to upper limit
        v = q(i) - obj.link(i).qlim(1);       % Proximity to lower limit
        dp = -u^-2 - v^-2;                    % Partial-derivative
        pdot = dp*qdot(i);                    % Time-derivative
        if pdot > 0
            ret(i,i) = 1/u + 1/v - 4/(obj.link(i).qlim(2) - obj.link(i).qlim(1)) + 1;
        end
    end
end

