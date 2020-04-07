%% Cubic Spline Trajectory
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

classdef CSpline < handle
    
    properties (Access = public)
        point;                                                              % m x n array of n points
        time;                                                               % 1 x n vector of time for each point
    end
    
    properties (Access = private)
        a,b,c,d;                                                            % Spline coefficients
        m;                                                                  % Dimensions
        n;                                                                  % No. of points
    end
    
    methods (Access = public)
        %%%%% Constructor %%%%%
        function obj = CSpline(t,p)
            if size(t,2) ~= size(p,2)
                error("Inputs are not of equal length. Expected a 1xn time vector, and mxn array for n points.")
            elseif size(t,2) < 3
                error("A minimum number of 3 points is needed for a cubic spline.")
            else
                n = size(p,2);
                for i = 1:n-1
                    if t(i) > t(i+1)
                        error("Time must be in ascending order. They have been automically sorted.")
                    end
                end
                obj.time = t;                                               % Assign time vector
                obj.point = p;                                              % Assign poisition array
                obj.m = size(p,1);                                          % Dimensions
                obj.n = size(p,2);                                          % No. of points
                
                A = zeros(n,n);
                B = zeros(n,n);
                
                dt = t(2) - t(1);
                A(1,1) = (dt^2)/3;
                A(1,2) = (dt^2)/6;
                B(1,1) = -1;
                B(1,2) = 1;
                dt = t(n) - t(n-1);
                A(n,n-1) = -dt^2/6;
                A(n,n) = -dt^2/3;
                B(n,n-1) = -1;
                B(n,n) = 1;

                for i = 2:n-1
                    dt1 = t(i) - t(i-1);
                    dt2 = t(i+1) - t(i);
                    A(i,i-1) = dt1/6;
                    A(i,i) = (dt1 + dt2)/3;
                    A(i,i+1) = dt2/6;
                    B(i,i-1) = 1/dt1;
                    B(i,i) = -1/dt1 - 1/dt2;
                    B(i,i+1) = 1/dt2;
                end
                
                C = A\B;

                % Solve spline coefficients
                obj.a = nan(obj.m,obj.n-1);
                obj.b = obj.a;
                obj.c = obj.b;
                obj.d = obj.c;
                
                for i = 1:obj.m
                    s = p(i,:)';
                    sdd = C*s;
                    for j = 1:n-1
                        dt = t(j+1) - t(j);
                        obj.a(i,j) = s(j);
                        obj.c(i,j) = 0.5*sdd(j);
                        obj.d(i,j) = (sdd(j+1) - sdd(j))/(6*dt);
                        
                        if j == 1
                            obj.b(i,1) = 0;
                        elseif j == n-1
                            obj.b(i,n-1) = -0.5*dt*(sdd(j+1) + sdd(j));
                        else
                            ds = s(j+1)-s(j);
                            obj.b(i,j) = ds/dt - dt*(sdd(j+1) + 2*sdd(j))/6;
                        end
                    end
                end
            end
        end
        
        function [pos, vel, acc] = getState(obj,t)
                pos = nan(obj.m,1);
                vel = zeros(obj.m,1);
                acc = zeros(obj.m,1);
                if t >= obj.time(end)
                    pos = obj.point(:,end);
                elseif t <= obj.time(1)
                    pos = obj.point(:,1);
                else
                    for k = 1:obj.n-1
                        j = obj.n - k;                                      % Count backwards
                        if t > obj.time(j)
                            break                                           % Break at the jth spline
                        end
                    end
                    dt = t - obj.time(j);                                   % Time since start of jth spline
                    for i = 1:obj.m
                        pos(i,1) = obj.a(i,j) + obj.b(i,j)*dt +   obj.c(i,j)*dt^2 +   obj.d(i,j)*dt^3;
                        vel(i,1) =              obj.b(i,j)    + 2*obj.c(i,j)*dt   + 3*obj.d(i,j)*dt^2;
                        acc(i,1) =                              2*obj.c(i,j)      + 6*obj.d(i,j)*dt;
                    end
                end
        end
    end
    
    methods (Static)
        function ret = sort(input)
            for i = 1:length(input)
                if input(i+1) < input(i)
                    temp = input(i);
                    input(i) = input(i+1);
                    input(i+1) = temp;
                end
            end
            ret = input;
        end
    end
end