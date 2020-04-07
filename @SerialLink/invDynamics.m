%% SerialLink.invDynamics()
% Jonathan Woolfrey
%
% Computes the joint torques required to control a serial link manipulator.



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

function ret = invDynamics(obj,acc,vel,Kd,pos,Kp)
    
    switch nargin
        case 2
            qddot = acc;
        case 4
            qddot = acc + Kd*(vel - obj.qdot);
        case 6
            qddot = acc + Kp*(pos - obj.q) + Kd*(vel - obj.qdot);
        otherwise
            error('Incorrect number of input arguments.');
    end
        
    tau = obj.M*qddot + obj.C*obj.qdot + obj.grav + obj.getJacobian'*[10;0;0;0;0;0];                          % Inverse dynamics calculation
    
   % Saturate any joint torques over the limit
   for j = 1:obj.n
       if abs(tau(j)) > obj.link(j).tlim
           tau(j) = sign(tau(j))*obj.link(j).tlim;
       end
   end
   
   ret = tau;

end