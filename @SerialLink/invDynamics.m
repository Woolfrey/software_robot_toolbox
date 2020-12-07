%% SerialLink.invDynamics()
% Jonathan Woolfrey
%
% Computes the joint torques required to control a serial link manipulator.
% 
% Inputs:
% - accel           Joint accelerations to be achieved (nx1)
% - vel             Current joint velocities (nx1) (optional)
% - pos             Current joint positions (nx1) (optional)
%
% The joint torques are given by:
%
% tau = M*acc + (C+D)*vel + g,
%
% where g is the gravitational torque vector. If the velocity and positions
% are given, the dynamics equation will be computed accordingly. Otherwise,
% the current manipulator state is used.
%
% This function will also saturate any joint torques that violate joint
% limits.
%
% TO DO:
%   - Add friction and payload forces to the torque equation


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

function ret = invDynamics(obj,acc, vel, pos)

    if nargin == 2              % Use current joint state
            vel = obj.qdot;
            pos = obj.q;
            M = obj.M;
            C = obj.C;
            g = obj.grav;
    elseif nargin == 4          % Use given joint state
            M = obj.getInertia(pos);
            C = obj.getCoriolis(pos,vel);
            g = obj.getGrav(pos);
    else
        error("Incorrect number of inputs. If only joint acceleration is given, this method will use the current joint state to compute the joint torques. Otherwise, acceleration, velocity, and position must be given.");
    end
    tau = M*acc + (C + obj.D)*vel + g;           
            
   % Saturate any joint torques over the limit
   for j = 1:obj.n
       if abs(tau(j)) > obj.link(j).tlim
           tau(j) = sign(tau(j))*obj.link(j).tlim;
       end
   end
   
   ret = tau;

end