%% MobileManipulator.updateState()
% Jon Woolfrey
% Work in progress.


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

%%% Update Kinematics and Dynamics
function updateState(obj,pose,vel,acc,q,qdot)
    obj.acc = acc;                                                          % Update base acceleration 
    obj.pose = pose;                                                        % Update location
    obj.vel = vel;                                                          % Update base velocity
    obj.arm.updateState(q,qdot,obj.pose*obj.baseTF);                        % Update manipulator kinematics, dynamics    
    
    % Inertial effects for the arm
    obj.cdot = obj.getCdot();
    obj.Mmb = obj.getMmb();                                                 % Maps base acceleration to joint torque
    obj.Cmb = obj.getCmb();                                                 % Maps base velocity to joint torque
    obj.torque = -obj.Mmb*obj.acc - obj.Cmb*obj.vel;                     	% Torque needed to oppose base motion effects
end