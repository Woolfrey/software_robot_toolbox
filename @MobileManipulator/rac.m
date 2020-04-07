%% MobileManipulator.rac()
% Jon Woolfrey
%
% Computes the end-effector control for a mobile manipulator and accounts
% for effects of base motion.
%
% This is a work in progress.


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

    r = obj.arm.tool.pos - obj.pose.pos;                                    % Distance from origin to end-effector
    Sa = skew(obj.acc);                                                     % Skew-symmetric matrix of angular acceleration
    Sw = skew(obj.vel);                                                     % Skew-symmetric matrix of angular velocity
    J = obj.arm.getJacobian();                                              % Manipulator Jacobian at current state
    xddot = acc - obj.acc - [(Sa + Sw^2)*r + 2*Sw*J(1:3,:)*obj.arm.qdot; zeros(3,1)];

    if nargin < 9
        qddr = zeros(obj.arm.n,1);                                       	% No redundant accelerations
    else
        xdot = vel - obj.vel - [Sw*r; zeros(3,1)];                      	% Compensation for base motion
    end
    if nargin < 6
        pos = Pose();                                                       % Pose is arbitrary
        Kp = 0;                                                             % No proportional feedback
        Ko = 0;
    end
    if nargin < 3
        xdot = zeros(6,1);                                                	% Desired velocity arbitrary
        Kv = 0;                                                             % No velocity feedback
        Kw = 0;
    end

    if nargin < 10
        ret = obj.arm.rac(xddot, xdot, Kv, Kw, pos, Kp, Ko, qddr);          % No redundant velocities specified
    else
        ret = obj.arm.rac(xddot, xdot, Kv, Kw, pos, Kp, Ko, qddr, qdr);    	% Redundant velocities specified
    end
        
end
