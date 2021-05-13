%% SerialLink.getAcc()
% Jonathan Woolfrey
%
% This function computes the joint accelerations from a given joint torque:
%
% qddot = M^-1(tau - C*qdot - D*qdot g),
%
% where:
% - qddot (nx1) is a vector of joint accelerations,
% - M  (nxn) is the inertia matrix,
% - tau (nx1) is the input joint torque vector
% - C (nxn) is the Coriolis matrix,
% - D (nxn) is the damping matrix, and
% - g (nx1) is the gravity torque vector.
%
% Inputs:
% - tau
% - qdot (optional)
% - q (joint positions, also optional)
%
% Output:
% - qddot

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

function ret = getAcc(obj,tau,disturbance)
 
	if nargin < 3
        disturbance = zeros(obj.n,1);
    end
    
    % Inertia matrix can be ill-conditioned, so need to add damping to the
    % inverse.
    d = det(obj.M);
    if d < 1E-5
        damping = (1-(d/1E-5)^2)*6E-4;          % This value seems to work well
        invM = obj.M/(obj.M*obj.M + damping*eye(obj.n));
    else
        invM = inv(obj.M);
    end
    
    ret = invM*(tau - (obj.C + obj.D)*obj.qdot - obj.grav - disturbance);
end