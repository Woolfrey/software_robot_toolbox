%% Rotation class
% Jonathan Woolfrey
% August 2019
%
% This class defines the relative orientation between references frames.
% The default representation is in quaternion space (H) due to its
% computational efficiency.


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

classdef Rotation < handle
    
    %%%%%%%%%% PROPERTIES %%%%%%%%%%
    properties (Access = public)
        quat;                                                               % Quaternion [w x y z];
    end
    
    %%%%%%%%%% METHODS %%%%%%%%%%
    methods (Access = public)
        
        %%%% Constructor(s)%%%
        function obj = Rotation(method,a,b,c)
            if nargin == 0
                obj.quat = [1 0 0 0]';                                      % Zero rotation
            elseif ~ischar(method)                                          % First input must be string or char
                error("Please specify the type of input as 'rpy', 'quat', or 'angleAxis'.");
            else
                switch method
                    case 'rpy'
                        if nargin == 2                                      % Euler angles given as a vector
                            R = Rotation.rpy2rot(a);                        % Get the SO(3) representation
                            obj.quat = Rotation.rot2quat(R);                % Convert to H
                            obj.quat = obj.quat/norm(obj.quat);             % Normalize
                        elseif nargin == 4                                  % Individual Euler angles
                            R = Rotation.rpy2rot(a,b,c);                    % Get the SO(3) representation
                            obj.quat = Rotation.rot2quat(R);                % Convert to H
                            obj.quat = obj.quat/norm(obj.quat);             % Normalize
                        else
                            error("Incorrect number of inputs for 'rpy'. Expected a 3-element vector, or 3 individual angles.");
                        end
                    case 'quat'
                        if nargin == 3
                            obj.quat = [a;b];
                            obj.quat = obj.quat/norm(obj.quat);             % Normalize
                        else
                            error("Incorrect number of inputs for 'quat'. Expected a scalar and a 3-element vector.");
                        end
                    case 'angleAxis'
                        if nargin == 3
                            obj.quat = [cos(0.5*a); sin(0.5*a)*b];
                            obj.quat = obj.quat/norm(obj.quat);             % Normalize
                        else
                            error("Incorrect number of inputs for 'angleAxis'. Expected a scalr and a 3-element vector.");
                        end
                    otherwise
                        error("Please specify the method as 'rpy', 'quat', or 'angleAxis', followed by the relevant inputs.");
                end
            end
        end
        
        % Quaternion multiplication
        function ret = mtimes(a,b)
            scalar = a.quat(1)*b.quat(1) - a.quat(2:4)'*b.quat(2:4);        % Scalar
            vector = a.quat(1)*b.quat(2:4) + b.quat(1)*a.quat(2:4) ...
                     + cross(a.quat(2:4),b.quat(2:4));                      % Vector
                 
            if scalar < 0
                scalar = -1*scalar;
                vector = -1*vector;
            end

            ret = Rotation('quat',scalar,vector);
        end
        
        %%% Declaration of functions in separate files %%%
        ret = angle(obj);                                                   % Get the angle of rotation
        ret = axis(obj);                                                    % Get the axis of rotation
        ret = error(obj,desired);                                           % Get the error between this rotation and another
        ret = inverse(obj);                                                 % Get the inverse of this rotation
        ret = matrix(obj);                                                  % Get the 3x3 rotation matrix
        plot(obj,origin);                                                   % Plot the reference frame represented by this rotation
        ret = rotate(obj,input);                                            % Rotate a 3x1 vector or 3x3 matrix
        ret = rpy(obj);                                                     % Returns the roll, pitch, and yaw angles of this rotation
        setAngleAxis(obj,angle,axis);                                       % Set this rotation via angle axis
        setRPY(obj,r,p,y);                                                  % Set this rotation via roll pitch yaw angles
    end
    
    methods (Static)
        
        %%% Declaration of functions in separate class files %%%
        ret = rot2quat(R);                                                  % Convert from rotation matrix to quaternion
        ret = rotx(roll);                                                   % Give the rotation matrix about the x axis
        ret = roty(pitch);                                                	% Give the rotation matrix about the y axis
        ret = rotz(yaw);                                                 	% Give the rotation matrix about the z axis
        ret = rpy2rot(r,p,y);                                               % Returns the rotation matrix from roll, pitch, and yaw angles
    end

end