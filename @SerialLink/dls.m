%% Damped-Least-Squares inversion of the Jacobian matrix
% Jonathan Woolfrey
%
% This function computes the damped-least-squares inverse of a matrix.



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

function ret = dls(obj,J,epsilon,lambdaMax,W,verbose)    
    
    if nargin < 5
        W = eye(obj.n);
        verbose = false;
    elseif nargin < 6
        verbose = false;
    elseif nargin == 6
        if strcmp(verbose,'verbose')
            verbose = true;
        else
            error("Incorrect spelling. Input 'verbose' as 5th argument to display information in the console.");
        end
    end
            
    mu = sqrt(det(J*J'));                                                   % Compute manipulability
        
    if mu < epsilon                                                         % Manipulability below threshold
        lambda = (1 - (mu/epsilon)^2)*lambdaMax;                            % Compute damping factor
        if verbose
            disp("Manipulator is near-singular!");
        end
    else
        lambda = 0;
    end
    invWJt = W\J';                                                          % Make calculations a little faster
    ret = invWJt/(J*invWJt + lambda*eye(6));
end