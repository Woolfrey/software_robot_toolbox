%% SerialLink.getAcc()
% Jonathan Woolfrey
%
% Get the joint accelerations for a given torque input.



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

function ret = getAcc(obj,tau,q,qdot)
    if nargin == 2
        qdot = obj.qdot;
        M = obj.M;
        C = obj.C;
        g = obj.grav;
    else
        M = obj.getInertia(q);
        C = obj.getCoriolis(q,qdot);
        g = obj.getGrav(q);
    end
    
    ret = M\(tau - C*qdot - g);
end