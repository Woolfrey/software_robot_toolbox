%% SerialLink.rmrc()
% Jonathan Woolfrey
%
% Resolved Motion Rate Control. This function computes the joint velocities
% required to track a given end-effector trajectory. The solution for the
% joint velocities is computed via weighted-least-squares:
%
%       qdot = invJ*xdot,
%
% where:
%
%       invJ = W^-1*J'*(J*W^-1*J')^-1,
%
% and W is a positive-definite weighting matrix. By default, this weighting
% matrix combines the inertia matrix with a penalty term based on proximity
% to the joint limits to enable joint limit avoidance.
%
% This function will also check for singularities and add damping to allow
% inversion of the Jacobian as necessary.



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

function ret = rmrc(obj,vel,pose,Kp,Ko,qdr)
    
    epsilon = 1E-2;                                                         % Threshold for activating DLS
    lambdaMax = 0.1;                                                        % Maximum damping value
    
    switch nargin
        case 2        
            e = zeros(6,1);                                                 % No feedback control
            Kp = 0;
            Ko = 0;
            qdr = zeros(obj.n,1);
        case 5
            e = obj.tool.error(pose);                                       % Current pose error
            qdr = zeros(obj.n,1);
        case 6
            e = obj.tool.error(pose);
    end
            
    xdot = vel + [Kp*e(1:3); Ko*e(4:6)];
    
    J = obj.getJacobian();                                                  % Compute Jacobian at current pose
    if obj.n > 6
        W = obj.M + obj.getJointWeight(obj.q,obj.qdot);                     % Weighted solution
        invJ = obj.dls(J,epsilon,lambdaMax,W);                              % Weighted pseuodinverse
        N = eye(obj.n) - invJ*J;                                            % Null space projection matrix
        qdot = invJ*xdot + N*qdr;                                           % rmrc with redundant task
    else
        invJ = obj.dls(J,epsilon,lambdaMax);
        qdot = invJ*xdot;
    end
    
    qdot = obj.saturateVelocities(obj.q,qdot,'scaled');                     % Obey joint constraints
    
    ret = qdot;
    
end