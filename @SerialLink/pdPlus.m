%% SerialLink.pdPlus()
% Jonathan Woolfrey
%
% PD+ Control of a Serial Link Manipulator. The joint control torques are
% given by:
%
%       tau = Kp*e + Kd*edot + C*qdot + g,
%
% where:
%   - e is the joint position tracking error,
%   - edot is the joint velocity tracking error,
%   - Kp, Kd are positive-definite gain matrices
%
%   Inputs:
%   - pos:  Desired joint position (nx1)
%   - Kp:   Gain matrix for position error (nxn)
%   - vel:  Desired joint velocity (nx1)
%   - Kd:   Gain matrix for velocity error (nxn)
%
%   Optional inputs q and qdot define a theoretical joint state to compute
%   the control. Otherwise, the function will use the current joint state
%   of the SerialLink object.



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

function ret = pdPlus(obj,vel,Kd,pos,Kp)
    if nargin == 3
        pos = zeros(obj.n,1);
        Kp = 0;
    end

    tau = Kp*(pos - obj.q) + Kd*(vel - obj.qdot) + obj.C*obj.qdot + obj.grav;
    
    % Saturate any joint torques over their limits
    for j = 1:obj.n
        if abs(tau(j)) > obj.link(j).tlim
            tau(j) = sign(tau(j))*obj.link(j).tlim;
        end
    end
    
    ret = tau;
end