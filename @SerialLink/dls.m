%% Damped-Least-Squares inversion of the Jacobian matrix
% Jonathan Woolfrey
%
% This function computes the damped-least-squares inverse of a matrix. For
% a matrix J, the DLS inverse is:
%
% J_inv = J'*(J*J' + lambda*eye(n))^-1,
%
% where n is the number of columns. Lambda is the damping parameter. It is
% autonomously computed using the threshold value and maximum damping value
% in the corresponding serial link class:
%
% lambda = (1 - (manipulability/threshold)^2)*max_damping.
%
% If the manipulability is above the threshold value, lambda = 0.

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

function ret = dls(obj,J,W,option)
    verbose = false;
    
    if nargin < 2
        J = obj.getJacobian();
    end
    if nargin < 3
        W = eye(obj.n);
    end
    if nargin == 4
        if strcmp(option,"verbose")
            verbose = true;
        else
            error("Incorrect spelling. Input 'verbose' as 5th argument to print information in the console.");
        end
    end
    
    mu = sqrt(det(J*J'));
    
    if mu < obj.threshold
        damping = (1 - (mu/obj.threshold)^2)*obj.maxDamping^2;
        if verbose
            warning("The manipulator is near singular!");
        end
    else
        damping = 0;
    end
    
    % N.B. inertia matrix can be poorly conditioned so need to damp it if
    % using it as the weighting matrix
    if det(W) < 1E-5
        invWJt = W'/(W*W' + 1E-4*eye(obj.n))*J';
    else
        invWJt = W\J';
    end
    ret = invWJt/(J*invWJt + damping*eye(6));

end