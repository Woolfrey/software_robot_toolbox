%% SerialLink.getInertia();
% Jonathan Woolfrey
%
% Computes the inertia matrix of the manipulator in the joint space. The
% joint torques for a manipulator are:
%
%       tau = M*qddot + C*qdot + g,
%
% where M (nxn) is the inertia matrix. By definition, the inertia matrix
% must be positive definite such that:
%
%               qdot'*M*qdot > 0   for all qdot.



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

function ret = getInertia(obj,q)
    ret = zeros(obj.n,obj.n);                                               % Pre-allocate memory
    if nargin == 1                                                          % No inputs, use current state
        K = obj.Jm;                                                         % Jacobian to c.o.m.
        I = obj.H;                                                          % Inertia of each link
    else
        K = obj.getMassJacobian(q);                                         % Compute Jacobian to c.o.m.
        I = obj.getLinkInertia(q);                                          % Compute the inertia for each link
    end
    for i = 1:obj.n
        ret = ret + obj.link(i).mass*K(1:3,:,i)'*K(1:3,:,i) + K(4:6,:,i)'*I(:,:,i)*K(4:6,:,i);
    end
end