%% Trapezoidal Velocity Profile
% Jonathan Woolfrey
% August 2019



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

classdef Trapezoidal < handle
    
    properties (Access = public)
        t0;                                                                 % Start time (s)
        tf;                                                                 % End time (s)
        p1;                                                                 % Start point
        p2;                                                                 % End point
    end
    
    properties (Access = private)
        t1, t2;                                                             % Intermediate times
        a,v;                                                                % Max acceleration, velocity
        theta;                                                              % Used in slerp
    end
    
    methods (Access = public)
        %%%%% Constructor %%%%%
        function obj = Trapezoidal(t0,tf,p1,p2)
            if size(p1,1) ~= size(p2,1) || size(p1,2) ~= size(p2,2)
                error("Inputs p1 and p2 must be of the same size.")
                return
            elseif tf < t0
                error("Input tf must be greater than t0.")
                return
            else
                obj.t0 = t0;
                obj.tf = tf;
                obj.p1 = p1;
                obj.p2 = p2;
                
                obj.v = 4/(3*tf - 3*t0);                                    % Maximum velocity
                obj.t1 = t0 + 1/(3*obj.v);                                  % Time to reach max. velocity                              
                obj.t2 = t0 + 1/obj.v;                                      % Time to start deceleration
                obj.a = obj.v/(obj.t1 - obj.t0);                            % Maximum acceleration
                
                % Used in SLERP
                if length(p1) == 4                                          % Assume quaternion
                    obj.theta = acos(p1'*p2/(norm(p1)*norm(p2)));           % Angle between vectors
                end
            end
        end
        
        %%%%% Get Functions %%%%%
        function [s, sd, sdd] = getScalar(obj,t)
            if t > obj.tf
                s = 1;
                sd = 0;
                sdd = 0;
            elseif t > obj.t2
                dt = t - obj.t2;
                s = 5/6 + dt*obj.v - 0.5*dt^2*obj.a;
                sd = obj.v - dt*obj.a;
                sdd = -obj.a;
            elseif t > obj.t1
                dt = t - obj.t1;
                s = 1/6 + dt*obj.v;
                sd = obj.v;
                sdd = 0;
            elseif t > obj.t0
                dt = t - obj.t0;
                s = 0.5*dt^2*obj.a;
                sd = dt*obj.a;
                sdd = obj.a;
            else % t < t0
                s = 0;
                sd = 0;
                sdd = 0;
            end
        end
        
        % Linear interpolation (LERP)
        function [pos, vel, acc] = lerp(obj,t)
            [s, sd, sdd] = obj.getScalar(t);
            [pos, vel, acc] = Trajectory.lerp(s,sd,sdd,obj.p1,obj.p2);
        end
        
        % Spherical Linear Interpolation (SLERP)
        function [pos, vel, acc] = slerp(obj,t)
            if length(obj.p1) ~= 4 || length(obj.p2) ~= 4
                error("slerp() is only for quaternion interpolation; expected a 4x1 vector for p1, p2");
            end
            [s, sd, sdd] = obj.getScalar(t);
            [pos, vel, acc] = Trajectory.slerp(s,sd,sdd,obj.p1,obj.p2,obj.theta);
        end
    end
    
end