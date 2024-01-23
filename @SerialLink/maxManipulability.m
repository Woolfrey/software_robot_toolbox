%% SerialLink.maxManipulability()
% Jonathan Woolfrey
%
% This function computes the gradient vector for the measure of
% manipulability, and multiplies it by a scalar. By projecting this vector
% on to the null space of a serial link manipulator, it will autonomously
% rearrange itself away from singular values.
%
% TO DO:
%   - Fill in code for optional joint position input

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

function ret = maxManipulability(obj,alpha)
    g = zeros(obj.n,1);                 % Memory allocation for gradient vector
    J = obj.getJacobian();              % Jacobian matrix
    invJ = obj.dls(J);                  % Pseudo-inverse Jacobian
    manipulability = sqrt(det(J*J'));
    for i = 2:obj.n
        dJdq = obj.getPartialJacobian(i);
        g(i) = manipulability*trace(dJdq*invJ); % Gradient of manipulability
    end
    ret = alpha*g;
end