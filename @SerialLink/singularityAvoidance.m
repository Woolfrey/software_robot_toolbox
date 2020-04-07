%% SerialLink.singularityAvoidance()
% Jonathan Woolfrey
%
% This function computes the task-reconstruction method for singularity
% avoidance.



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

function ret = singularityAvoidance(obj,qdot)
    epsilon = 1E-2;
    
    if nargin == 1
    else
        % Do something here
    end
    
    dJdq = obj.getPartialJacobian();                                        
    J = obj.J;
    JJt = J*J';
    m = sqrt(det(JJt));
    invJ = J'/(JJt);
    
    g = zeros(obj.n,1); % Gradient vector
    
    for i = 1:obj.n
        g(i) = m*trace(dJdq(:,:,i)*invJ);                                   % Compute gradient vector
    end
    
    mdot = qdot'*g;                                                         % Time-derivative
    if mdot < 0 && m <= epsilon
        a = -(g/norm(g));
        ret = qdot - (qdot'*a)*a;
    else
        ret = qdot;
    end

end