%% SerialLink.getCoriolis()
% Jonathan Woolfrey
%
% This function returns the matrix which maps joint velocities to joint
% torques. The total joint torque equation is:
%
%       tau = M*qddot + C*qdot + g
%
% where C*qdot (nx1) is the vector of torques from Coriolis and centripetal
% forces, and C (nxn) is the Coriolis matrix.
%
% TO DO:
%   - Optional inputs for joint position q and velocity qdot


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

function ret = getCoriolis(obj)
        ret = zeros(obj.n,obj.n);                                           % Pre-allocate memory
        if nargin == 1                                                      % Use current state
            K = obj.Jm;                                                     % Jacobian to centre of mass
            Kdot = obj.Jmdot;                                               % Time-derivative
            I = obj.H;                                                      % Link inertia
            Idot = obj.Hdot;                                                % Time-derivative of link inertia
        else
            %%% Need to fill this in %%%%
        end
        for i = 1:obj.n
            ret = ret + obj.link(i).mass*K(1:3,:,i)'*Kdot(1:3,:,i) ...
                + K(4:6,:,i)'*(I(:,:,i)*Kdot(4:6,:,i) + Idot(:,:,i)*K(4:6,:,i));
        end
    end