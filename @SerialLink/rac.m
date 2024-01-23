%% SerialLink.rac()
% Jonathan Woolfrey
%
% Resolved acceleration control. This function computes the joint
% torques needed to move the end-effector at a desired acceleration.
%
% Inputs:
% - acc (6x1)           Desired end-effector acceleration
% - vel (6x1)           Desired end-effector velocity
% - Kd (1x1)            Gain on the velocity error
% - pos (Pose object)   Desired end-effector position
% - Kp (1x1)        	Gain on the end-effector pose error
% - redundant (nx1)     Joint accelerations relating to a redundant task
%
% Output:
% - Joint torques (nx1)

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

function ret = rac(obj, acc, vel, Kd, pos, Kp, redundant)

	J = obj.getJacobian();                                      % Manipulator Jacobian
    Jdot = obj.getJdot();                                       % Time-derivative of Jacobian
    
    if nargin == 2
        xddot = acc;                                            % Only acceleration defined
    elseif nargin == 3
        edot = vel - J*obj.qdot;                                % Velocity tracking error
        xddot = acc + Kd*edot;                                  % Proportional feedback
    elseif nargin > 3
        e = obj.tool.error(pos);                                % Position tracking error
        edot = vel - J*obj.qdot;                                % Velocity tracking error
        xddot = acc + Kp*e + Kd*edot;                           % End-effector acceleration
    end
        if obj.n > 6                                            % Kinematically redundant
            invJ = obj.dls(J,obj.M,"verbose");                  	% Use inertia weighting
        if nargin < 5                               
            qddot = invJ*(xddot - Jdot*obj.qdot);               % No null space control
        else
            N = eye(obj.n) - invJ*J;                            % Null space projection matrix
            qddot = invJ*(xddot - Jdot*obj.qdot) + N*redundant; % Null space accelerations defined
        end
    else
        invJ = obj.dls(J,eye(obj.n),"verbose");
        qddot = invJ*(xddot - Jdot*obj.qdot);
    end
    ret = obj.invDynamics(qddot);                               % Return joint torques    
end