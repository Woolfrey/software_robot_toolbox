%% SerialLink.getGrav()
% Jonathan Woolfrey
%
% This function computes the joint torques required for a serial link
% manipulator to oppose gravity. The total joint torque is:
%
%       tau = M*qddot + C*qdot + g,
%
% where g (nx1) is the gravitational torque vector.
%
% This function assumes that gravitational acceleration is defined as -9.81
% m/s/s in the z-direction of a global reference frame. The base pose of
% the manipulator is taken into account when computing the gravitational
% torque.

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

function ret = getGrav(obj,q,baseTF,g)

    switch nargin
        case 1
            g = [0;0;-9.81];
            J = obj.Jm;
        case 2
            g = [0;0;-9.81];
            J = obj.getMassJacobian(q);
        case 3
            g = [0;0;-9.81];
            J = obj.getMassJacobian(q,baseTF);
        case 4
            J = obj.getMassJacobian(q,baseTF);
        otherwise
            error("Incorrect number of inputs.");
    end

    ret = zeros(obj.n,1);                                                   % Pre-allocate memory
    for i = 1:3
        for j = 1:obj.n
            for k = 1:obj.n
                ret(j) = ret(j) - J(i,j,k)*g(i);                            % Torques equal and opposite to acceleration vector
            end
        end
    end

end