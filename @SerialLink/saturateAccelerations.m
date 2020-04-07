%% SerialLink.saturateAccelerations()
% Jon Woolfrey
%
% Saturate the joint accelerations to obey joint speed and position limits.



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

function ret = saturateAccelerations(obj,q,qd,qdd)
    dt = 1/obj.hertz;
    for i = 1:obj.n
        qU = min(2*obj.hertz*obj.hertz*(obj.link(i).qlim(2) - q(i) - dt*qd(i)),...
                 obj.hertz*(obj.link(i).vlim - qd(i)));
        qL = max(2*obj.hertz*obj.hertz*(obj.link(i).qlim(1) - q(i) - dt*qd(i)),...
                 -obj.hertz*(obj.link(i).vlim + qd(i)));
              
        if qdd(i) > qU
            qdd(i) = 0.9*qU;
            disp(['Control is violating limits of joint ', num2str(i),'!']);
        elseif qdd(i) < qL
            qdd(i) = 0.9*qL;
            disp(['Control is violating limits of joint ', num2str(i),'!']);
        end
        
    end
    ret = qdd;
end