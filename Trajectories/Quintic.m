%% Minimum Jerk Trajectory
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

classdef Quintic
    
    %%%%%%%%%% PROPERTIES  %%%%%%%%%%
    properties (Access = public)
        t0;                                                                 % Initial Time
        tf;                                                                 % Final Time
        p1;                                                                 % Initial Position
        p2;                                                                 % Final Position
    end
    
    properties (Access = private)
        a, b, c;                                                            % Polynomial coefficients
        theta;                                                              % For use in SLERP
    end
    
    %%%%%%%%%% METHODS  %%%%%%%%%%
    methods (Access = public)
        
        %%%% Constructor(s) %%%%%
        function obj = Quintic(t0,tf,p1,p2)
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
                
                obj.a = 6*(tf-t0)^-5;
                obj.b = -15*(tf-t0)^-4;
                obj.c = 10*(tf-t0)^-3;
                
                % Used in SLERP
                if length(p1) == 4                                          % Assume quaternion
                    obj.theta = acos(p1'*p2/(norm(p1)*norm(p2)));
                end
            end
        end

        %%%%%% Get Functions %%%%%
        function [s,sd,sdd] = getScalar(obj,t)
            if t > obj.tf
                s = 1;
                sd = 0;
                sdd = 0;
            elseif t > obj.t0
                dt = t - obj.t0;
                s   =    obj.a*dt^5 +    obj.b*dt^4 +   obj.c*dt^3;
                sd  =  5*obj.a*dt^4 +  4*obj.b*dt^3 + 3*obj.c*dt^2;
                sdd = 20*obj.a*dt^3 + 12*obj.b*dt^2 + 6*obj.c*dt;
            else
                s = 0;
                sd = 0;
                sdd = 0;
            end
        end
        
        function [pos, vel, acc] = lerp(obj,t) % Linear Interpolation
            [s, sd, sdd] = obj.getScalar(t);
            pos = (1-s)*obj.p1 + s*obj.p2;
            vel = sd*(obj.p2 - obj.p1);
            acc = sdd*(obj.p2 - obj.p1);
        end
        
        function [pos, vel, acc] = slerp(obj,t) % Spherical Linear Interpolation
            if length(obj.p1) ~= 4 || length(obj.p2) ~= 4
                error("slerp() is only for quaternion interpolation; expected a 4x1 vector for p1, p2");
            end
            [s, sd, sdd] = obj.getScalar(t);
            [pos, vel, acc] = Trajectory.slerp(s,sd,sdd,obj.p1,obj.p2,obj.theta);
        end
    end
end