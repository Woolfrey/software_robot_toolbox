%% SerialLink.rac()
% Jonathan Woolfrey
%
% Resolved acceleration control.



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

function ret = rac(obj, acc, vel, Kv, Kw, pos, Kp, Ko, qddr, qdr)

    lambdaMax = 0.1;                                                        % Maximum damping in DLS
    epsilon = 1E-2;                                                         % Threshold value for activating DLS

    if nargin < 9                                                           % No redundant accelerations
        qddr = zeros(obj.n,1);                                              
    end
    if nargin < 6                                                           % No desired position given
        e = zeros(6,1);                                                     % Pose error is arbtirary
        Kp = 0;                                                             % No proportional gain
    else
        e = obj.tool.error(pos);                    
        J = obj.getJacobian();                                              % Jacobian at current state
    end
    if nargin < 3                                                           % End-effector velocity is arbitrary
        edot = zeros(6,1);
        Kv = 0;                                                             % No derivative gain
    else
        edot = vel - J*obj.qdot;                                            % Velocity tracking error
    end

    xddot = acc + [Kp*e(1:3); Ko*e(4:6)] + [Kv*edot(1:3); Kw*edot(4:6)];  	% Required end-effector acceleration
    
    Jdot = obj.getJdot();                                                   % Get time-derivative at current state
    
    if obj.n > 6                                                            % Redundant manipulator
        W = obj.M + obj.getJointWeight(obj.q,obj.qdot);                     % Inertia + joint limit weighting
    else
        W = eye(obj.n);                                                     % No weighting
    end
    invJ = obj.dls(J,epsilon,lambdaMax,W);                                  % Compute inverse Jacobian
    
    qddot = invJ*(xddot - Jdot*obj.qdot);                                   % Compute joint accelerations
    
    if nargin > 8                                                           % Null space tasks given
        N = eye(obj.n) - invJ*J;                                            % Null space projection matrix
        switch nargin
            case 9
                qddot = qddot + N*qddr;                                     % Just null space acceleration
            case 10
                qddot = qddot + N*(qddr + Jdot'*invJ'*(obj.qdot - qdr));    % Null space acceleration and velocity
        end
    end
    
    ret = qddot;
%     ret = obj.saturateAccelerations(obj.q, obj.qdot, qddot);
      
end