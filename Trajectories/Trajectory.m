%% Trajectory Class
% Jonathan Woolfrey
% October 2019
%
% This is a static class with common functions for both the Quintic and
% Trapezoidal trajectory objects.



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

classdef Trajectory
    methods (Static)
        
        % Linear interpolation
        function [pos, vel, acc] = lerp(s,sd,sdd,p1,p2)                     
            pos = (1-s)*p1 + s*p2;
            vel = sd*(p2 - p1);
            acc = sdd*(p2 - p1);
        end
        
        
        % Spherical Linear Interpolation; for quaternions
        function [pos, vel, acc] = slerp(s,sd,sdd,p1,p2,theta)             
            if length(p1) ~= 4 || length(p2) ~= 4
                error("slerp() is only used for quaternions; expected a 4x1 vector for inputs p1, p2.");
            end
            if theta == 0                                                   % No angle = no interpolation
                pos = p1;
                vel = zeros(3,1);
                acc = zeros(3,1);
            else
                x = (1-s)*theta;                                         	% This assignment makes calcs easier
                y = s*theta;                                              	% This assignment makes calcs easier
                denom = sin(theta);                                        	% Denominator for slerp
                pos = (sin(x)*p1 + sin(y)*p2)/denom;                       	% Give the position            
                dpds = (-cos(x)*p1 + cos(y)*p2)*(theta/denom);              % Partial derivative of interpolation function w.r.t. s
                qdot = dpds*sd;                                           	% Quaternion velocity?
                qddot = -pos*theta^2*sd + dpds*sdd;                         % Quaternion acceleration?
                A = 2*[-pos(2:4), pos(1)*eye(3) + skew(pos(2:4))];        	% Matrix mapping to angular velocity
                Adot = 2*[-qdot(2:4), qdot(1)*eye(3) + skew(qdot(2:4))];   	% Time derivative
                vel = A*qdot;                                             	% Angular velocity
                acc = A*qddot;                                              % Angular acceleration N.B Adot*qdot = 0
            end
        end
        
    end
end