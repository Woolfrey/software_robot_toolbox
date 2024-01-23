%% SerialLink.rmrc()
% Jonathan Woolfrey
%
% Resolved motion rate control. This function computes the joint torques
% needed to move the end-effector at a desired velocity.
%
% Inputs:
% - vel (6x1)           Desired end-effector velocity
% - pos (Pose object)   Desired end-effector position
% - Kp (1x1)            Gain on the end-effector position
% - redundant (nx1)     Joint velocities relating to a redundant task
%
% Output:
% - Joint velocities (nx1)

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

function ret = rmrc2(obj,vel,pose,Kp,redundant)
    
    if nargin == 2
        xdot = vel;
    elseif nargin > 2
        e = obj.tool.error(pose);
        xdot = vel + Kp*e;
    end
    
    J = obj.getJacobian();                                	% Compute Jacobian at current pose
   
    if obj.n > 6
        W = obj.M + obj.getJointWeight(obj.q,obj.qdot); 	% Weighted solution
        invJ = obj.dls(J,W);                                % Weighted pseuodinverse
        if nargin < 5                                       % No redundant task specified
            qdot = invJ*xdot;
        else
            N = eye(obj.n) - invJ*J;                      	% Null space projection matrix
            qdot = invJ*xdot + N*redundant;                 % rmrc with redundant task
        end
    else
        invJ = obj.dls(J);                                
        qdot = invJ*xdot;
    end
    
    qdot = obj.saturateVelocities(qdot);                    % Slow any joints that will hit a limit
    
    ret = obj.invDynamics(0.95*obj.hertz*(qdot - obj.qdot));    % Return joint torques
    
end