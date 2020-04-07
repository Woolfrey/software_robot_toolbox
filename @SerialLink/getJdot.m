%% SerialLink.getJdot()
% Jonathan Woolfrey
%
% This function computes the time-derivative of the Jacobian matrix. The
% acceleration equations for a serial-link manipulator are given by:
%
%           xddot = J*qddot + Jdot*qdot,
%
% where J is the Jacobian and Jdot is its time-derivative. This matrix is
% required for resolved acceleration control and impedance control.



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

function ret = getJdot(obj,q,qdot,baseTF)
    ret = zeros(6,obj.n);                                                   % Pre-allocate memory
    if nargin == 1                                                          % No inputs, use current state
        v = obj.a;                                                          % Vector for axis of actuation
        vdot = obj.adot;                                                    % Time-derivative
        d = obj.r;                                                          % Distance from joint to end-effector
        ddot = obj.rdot;                                                    % Time-derivative
    else
        if nargin == 2
            qdot = zeros(obj.n,1);
            warning("Joint velocities assumed to be zero.")
            baseTF = obj.base;
%             warning("Evaluating Jacobian derivative at current base pose.");
        elseif nargin == 3
            baseTF = obj.base;
%             warning("Evaluating Jacobian derivative at current base pose.");
        end
        [~,FK] = obj.fk(q,baseTF);                                          % Get forward kinematics
        v = obj.getAxis(FK);                                                % Axis of actuation
        d = obj.getAxis(FK);                                                % Distance to end-effector
        w = obj.getOmega(v,qdot);                                           % Angular velocities
        vdot = obj.getAxisDot(v,w);                                         % Time-derivative of axis
        ddot = obj.getDistDot(d,w);                                         % Time-derivative of distance
    end
    for i = 1:obj.n
        if obj.link(i).isrevolute                                           % Revolute joint
            ret(:,i) = [ cross(vdot(:,i),d(:,i)) + cross(v(:,i),ddot(:,i))
                         vdot(:,i)];
        elseif  ~obj.link(i).isrevolute                                     % Prismatic joint
            ret(1:3,i) = vdot(:,i);
        else
            error("Joint type incorrectly specified. Cannot compute time-derivative of the Jacobian.");
        end
    end
end