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

function ret = getJointWeight(obj,q)
    
    ret = eye(obj.n);                                           % Joint weighting matrix to return
    for i = 1:obj.n
        u = obj.link(i).qlim(2);
        l = obj.link(i).qlim(1);
        upper = u - q(i);                                       % Distance to upper limit
        lower = q(i) - l;                                       % Distance from lower limit
        range = u - l;
        dpdq = range^2*(2*q(i) - u - l)/(4*upper^2*lower^2);    % Partial derivative
        
        if(dpdq*obj.qdot(i) > 0)                                % Moving toward a limit
            ret(i,i) = range^2/(4*upper*lower);                 % Penalize joint motion
        end
    end
end

