%% Serial Link Class
% Jonathan Woolfrey
% August 2019
%
% This class defines an object of multiple Link objects attached in
% sequence. A serial link manipulator is created by passing an array of
% Link objects as an input:
%
% robot = SerialLink([Link1 Link2 Link3 ... Linkn ])
%
% TO DO:
%   - Need to update dynamics to account for prismatic joints
%   - Need fix forward kinematics to account for tool offset

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

classdef SerialLink < handle
    
    %%%%%%%%%% PROPERTIES %%%%%%%%%%
    properties (Access = public)
        base;                      	% Base pose
        basecolors = [];          	% For 3D modeling        
        basefaces = [];           	% For 3D modeling
        basevertices = [];         	% For 3D modeling
        baseVelocity = zeros(3,1);  % This is used for mobile manipulators
        C;                          % Coriolis matrix
        D;                          % Joint torque damping matrix
        grav;                       % Gravitational torque
        hertz = 100;                % Default control frequency
        link;                       % Array of Link objects
        M;                          % Inertia matrix
        maxDamping = 0.2;           % Maximum damping to apply for Damped Least Squares
        n;                          % No. of joints
        name = "robot";             % Unique identifier
        payload = Payload();        % Payload object
        q;                          % Joint positions
        qdot;                       % Joint velocities
        threshold = 0.1;            % Threshold value for activating Damped Least Squares
        tool = Pose();              % Tool transform
    end
     
    properties (Access = private)
        a;                          % Axis of rotation for each joint
        adot;                     	% Time-derivative
        com;                        % Centre of mass for each link
        fkchain;                	% Forward kinematic chain
        H;                        	% Inertia for each link
        Hdot;                    	% Time-derivative of inertia for each link
        Jm;                         % Jacobian to c.o.m. for each link
        Jmdot;                     	% Time-derivative
        omega;                    	% Cumulative angular velocity
        r;                         	% Distance to end-effector
        rdot;                      	% Time-derivative
    end
    
    %%%%%%%%%% METHODS %%%%%%%%%%
    methods (Access = public)
        
        %%%%% Constructor %%%%%
        function obj = SerialLink(input)
            if nargin == 1
                obj.link = input;                           % Array of link objects                                       
                obj.n = length(input);                    	% No. of joints
                obj.q = zeros(obj.n,1);                   	% Vector of joint positions
                obj.qdot = zeros(obj.n,1);                	% Vector of joint velocities
                baseTF = Pose();                          	% Create default pose object
                obj.updateState(obj.q,obj.qdot,baseTF); 	% Fill in all initial values
                D = zeros(obj.n,obj.n);                     % Pre-allocate memory for damping matrix
                for i = 1:obj.n
                    D(i,i) = obj.link(i).damping;           % Fill in joint damping matrix
                end
            else
                error("Require an array of Link objects.")
            end
        end
        
        % Update State
        function updateState(obj,q,qdot,baseTF,baseVel)            
            if nargin == 2                                                  % No joint velocities, base pose given
                warning("Joint velocities assumed to be zero.");
                qdot = zeros(obj.n,1);                                      % Set velocities to zero
                baseTF = obj.base;                                          % Use current base pose
            elseif nargin == 3                                              % No base pose given
                baseTF = obj.base;                                          % Use current base pose
            elseif nargin == 4
                obj.base = baseTF;                                          % Update base pose
            elseif nargin == 5
                obj.base = baseTF;
                obj.baseVelocity = baseVel;
            end
            
            % Update Kinematics
            obj.q = q;                              	% Assign joint positions
            obj.qdot = qdot;                        	% Assign joint velocities
            [obj.tool, obj.fkchain] = obj.fk(obj.q,obj.base);
            obj.a = obj.getAxis();                      % Relies on FK
            obj.r = obj.getDist();                    	% Relies on FK
            obj.omega = obj.getOmega();               	% Relies on qdot, a
            obj.adot = obj.getAxisDot();              	% Relies on omega, a
            obj.rdot = obj.getDistDot();            	% Relies on omega, r, qdot, a
            
            % Update Dynamics
            obj.com = obj.getCOM();                   	% Relies on FK
            [obj.H, obj.Hdot] = obj.getLinkInertia(); 	% Relies on FK, omega
            obj.Jm = obj.getMassJacobian();            	% Relies on FK, c.o.m.
            obj.M = obj.getInertia();                  	% Relies on Jm, H
            obj.grav = obj.getGrav();                 	% Relies on Jm
            obj.Jmdot = obj.getMassJdot();            	% Relies on ???
            obj.C = obj.getCoriolis();                 	% Relies on Jm, Jmdot, omega
            if obj.payload.exists
                obj.payload.updateState(obj.a,obj.adot,obj.omega,obj.fkchain);
                obj.M = obj.M + obj.payload.getInertia();
                obj.C = obj.C + obj.payload.getCoriolis();
                obj.grav = obj.grav + obj.payload.getGrav();
            end
        end
        
        % Forward Declarations
        ret = dls(obj,J,W,verbose);                     % Damped Least Squares inverse
        ret = getAcc(obj,tau,disturbance);           	% Convert torque to acceleration
        vellipse(obj,q);                              	% Velocity ellipsoid
        fellipse(obj,q);                              	% Force ellipsoid
    end
    
    methods (Access = private)
        
        % Get axis of actuation for each joint in the origin frame
        function ret = getAxis(obj,FK,baseTF)
            ret = zeros(3,obj.n);                           % Pre-allocate memory
            if nargin == 1                              	% No inputs, use current state
                FK = obj.fkchain;
                baseTF = obj.base;
            elseif nargin == 2
                baseTF = obj.base;
            end
            temp = baseTF.rot.matrix;                    	% Get rotation matrix of base
            ret(:,1) = temp(1:3,3);                        	% First joint aligned with z-axis of base
            for i = 2:obj.n
                temp = FK(i-1).rot.matrix;                 	% Get the rotation matrix of preceding link
                ret(:,i) = temp(1:3,3);                   	% Joint i aligned with z axis of link i-1
            end
        end
        
        % Get the time-derivative for each axis of actuation
        function ret = getAxisDot(obj,axis,omega)
            ret = zeros(3,obj.n);                          	% Pre-allocate memory
            if nargin == 1                                  % No inputs, use current state
                axis = obj.a;
                omega = obj.omega;
            end
            for i = 1:obj.n
                ret(:,i) = cross(omega(:,i),axis(:,i));
            end
        end
                
        % Get the distance from each joint to the end-effector
        function ret = getDist(obj,FK)
            ret = zeros(3,obj.n);                           % Pre-allocate memory
            if nargin == 1                                	% No inputs, use current state
                FK = obj.fkchain;
            end
            ret(:,1) = FK(obj.n).pos;                     	% First link
            for i = 2:obj.n
                ret(:,i) = FK(obj.n).pos - FK(i-1).pos;    	% ith joint is aligned with link i-1
            end
        end
        
        % Get the time-derivative of distance from joint to end-effector
        function ret = getDistDot(obj,d,w,v,vel)
            ret = zeros(3,obj.n);                                           % Pre-allocate memory
            if nargin == 1                                                  % No inputs, use current state
                d = obj.r;                                                  % Distance from joint to end-effector
                w = obj.omega;                                              % Angular velocities
                v = obj.a;                                                  % Vector denoting axis of actuation
                vel = obj.qdot;                                             % Joint velocities                                  
            end
            ret(:,obj.n) = cross(w(:,obj.n),d(:,obj.n));                    % First joint
            if obj.link(obj.n).isrevolute == false                          % Prismatic joint
                ret(:,obj.n) = ret(:,obj.n) + vel(obj.n)*v(:,obj.n);        % Add effect of linear actuation
            end
            for i = 2:obj.n
                j = obj.n - i + 1;                                          % Count backwards
                l = d(:,j) - d(:,j+1);                                      % Link length
                ret(:,j) = cross(w(:,j),l) + ret(:,j+1);                    % Effect of proceeding link
                if obj.link(j).isrevolute == false                          % Prismatic joint
                    ret(:,j) = ret(:,j) + vel(j)*v(:,j);                    % Add effect of linear actuation
                end
            end
        end
        
        % Get cumulative angular velocity up the kinematic chain
        function ret = getOmega(obj,axis,vel)
            ret = zeros(3,obj.n);                                           % Pre-allocate memory
            if nargin == 1
                vel = obj.qdot;
                axis = obj.a;
            end
            if obj.link(1).isrevolute                                       % Revolute joint
                ret(:,1) = obj.baseVelocity + vel(1)*axis(:,1);                                % Angular velocity
            end
            for i = 2:obj.n
                ret(:,i) = ret(:,i-1);                                      % Add previous velocity
                if obj.link(i).isrevolute                                   % Revolute joint
                    ret(:,i) = ret(:,i) + vel(i)*axis(:,i);                 % Accumulate angular velocity
                end
            end
        end
        
        % Get the location for the centre of mass for each link
        function ret = getCOM(obj,FK)
            ret = zeros(3,obj.n);                                           % Pre-allocate memory
            if nargin == 1                                                  % No inputs, use current state
                FK = obj.fkchain;
            end
            
            % These forward declaration make calcs a little faster
            c = cos(obj.q(1));
            s = sin(obj.q(1));
            
            % Centre of mass = p + R*R(q)*c, where:
            % p is the position of the previous link transform,
            % R is the rotation of the previous link transform,
            % R(q) is the offset from the joint rotation,
            % c is the local position for the centre of mass
            ret(:,1) = obj.base.pos + obj.base.rot.matrix*...
                       [obj.link(1).com(1)*c - obj.link(1).com(2)*s
                        obj.link(1).com(1)*s + obj.link(1).com(2)*c
                        obj.link(1).com(3)];
                    
            for i = 2:obj.n
                c = cos(obj.q(i));
                s = sin(obj.q(i));
                ret(:,i) = FK(i-1).pos + FK(i-1).rot.matrix*...
                           [obj.link(i).com(1)*c - obj.link(i).com(2)*s
                            obj.link(i).com(1)*s + obj.link(i).com(2)*c
                            obj.link(i).com(3)];
            end              
        end
        
      	% Compute Jacobian for the centre of mass for every link
        function ret = getMassJacobian(obj,q,baseTF)
            ret = zeros(6,obj.n,obj.n);                                     % Pre-allocate memory
            if nargin == 1                                                  % No inputs, use current state
                FK = obj.fkchain;                                           % Forward kinematics chain
                c = obj.com;                                                % Location for each centre of mass
                v = obj.a;                                                  % Vector denoting axis for each link
            else
                if nargin == 2
                    baseTF = obj.base;                                      % Use current base pose
                end
                [~,FK] = obj.fk(q,baseTF);                                  % Forward kinematics chain
                v = obj.getAxis(FK,baseTF);                                 % Vector for each joint axis
                c = obj.getCOM(FK);                                         % Location for each link c.o.m.
            end
            
            for i = 1:obj.n 
                for j = 1:i
                    if j == 1
                        d = c(:,i); 
                    else
                        d = c(:,i) - FK(j-1).pos;                           % Distance from jth joint to ith c.o.m.
                    end
                    if obj.link(j).isrevolute
                        ret(:,j,i) = [cross(v(:,j),d); v(:,j)];             % Effect of revolute joint
                    else
                        ret(1:3,j,i) = v(:,j);                              % Effect of prismatic joint
                    end
                end
            end
        end
        
        % Compute the time-derivative for the c.o.m. Jacobian
        function ret = getMassJdot(obj)
            ret = zeros(6,obj.n,obj.n);
            if nargin == 1
                v = obj.a;
                vdot = obj.adot;
                c = obj.com;
                w = obj.omega;
                FK = obj.fkchain;
                vel = obj.qdot;
                ddot = obj.rdot;
            else
                %%% Need to fill this in %%%
            end
            for i = 1:obj.n                                                 % Cycle through every link
                % Compute distance from joint i to c.o.m. of link i
                if i == 1
                    s = c(:,1);
                else
                    s = c(:,i) - FK(i-1).pos;
                end
                % Compute time-derivative of this value
                sdot = cross(w(:,i),s);
                if obj.link(i).isrevolute == false                          % Prismatic
                    sdot = sdot + vel(i)*v(:,i);                            % Add effect of linear actuator
                end
                for j = 1:i                                                 % Compute effect of jth joint up to ith link
                    if obj.link(j).isrevolute                              	% Revolute
                        % Compute distance from joint j to c.o.m. for link i
                        if j == 1
                            t = c(:,i);
                        else
                            t = c(:,i) - FK(j-1).pos;                    	% N.B. joint j is aligned with link j-1
                        end
                        tdot = ddot(:,j) - ddot(:,i) + sdot;                % Time-derivative with some clever math
                        ret(:,j,i) = [cross(vdot(:,j),t) + cross(v(:,j),tdot)
                                      vdot(:,j)];
                    elseif obj.link(j).type == 1
                        ret(1:3,j,i) = vdot(:,j);
                    else
                        error("Joint type incorrectly specified. Cannot compute dynamics.");
                    end
                end
            end
        end
        
        % Get the inertia matrices for each link in the origin frame
        function [I,Idot] = getLinkInertia(obj,q)
            I = zeros(3,3,obj.n);                           % Pre-allocate memory
            Idot = zeros(3,3,obj.n);                        % Pre-allocate memory
            if nargin == 1                                  % Use current state
                FK = obj.fkchain;                           
                w = obj.omega;
                q = obj.q;
            else                                            % Compute for given joint state
                [~,FK] = obj.fk(q);
                w = zeros(3,obj.n);
            end
            
            % This makes things a little faster
            c = cos(q(1));
            s = sin(q(1));
            
            % Orientation of link from base and joint rotation
            R = obj.base.rot.matrix*[c, -s, 0
                              s,  c, 0
                              0,  0, 1];
                          
            I(:,:,1) = R*obj.link(1).inertia*R';        % Rotate inertia for first link
            
            for j = 2:obj.n
                c = cos(q(j));
                s = sin(q(j));
                
                R = FK(j-1).rot.matrix*[c, -s, 0
                                        s,  c, 0
                                        0,  0, 1];
                                    
                I(:,:,j) = R*obj.link(j).inertia*R';           % Rotate the inertia matrix to the origin frame
                Idot(:,:,j) = skew(w(:,j))*I(:,:,j);           % Compute the time-derivative
            end
        end  
    end
end