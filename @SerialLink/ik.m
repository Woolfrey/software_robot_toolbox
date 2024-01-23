%% SerialLink.ik()
% Jonathan Woolfrey
%
% This function will solve the inverse kinematics of a serial link
% manipulator for a desired end-effector pose.
%
% By default, this algorithm uses the Jacobian inverse method as it appears to
% be faster:
%
%           q(k+1) = q(k) + alpha*inv(J)*e
%
% However, the transpose method can be specified as an option. This does
% not suffer from instability at singularities.
%
% Inputs:
%   - desired:      The desired end-effector pose (Pose object)
%   - q0:           Initial guess for the joint configuration (nx1).
%                   Default is randomized.
% Output:
%   - q:            The optimized joint configuration (nx1).



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

function ret = ik(obj,desired,q0,varargin)        
	% Default parameters
    animate = false;
	method = 'inverse';
	minstep = 1E-4;
	origin = obj.base;
	steps = 1E3;
	W = diag([1 1 1 0.05 0.05 0.05]);                                   	% Default weighting matrix                             
        
    if nargin == 2                                                        	% No initial guess given
        q0 = 0.2*randn(obj.n,1);                                        	% Randomize an initial guess
    end

    % Process optional inputs
    for i = 1:length(varargin)
        option = varargin{i};
        if ischar(option)
            switch option
                case 'animate'
                    animate = true;
                case 'method'
                    method = varargin{i+1};
                case 'minstep'
                    minstep = varargin{i+1};
                case 'origin'
                    origin = varargin{i+1};
                case 'steps'
                    steps = varargin{i+1};
                case 'weight'
                    W = varargin{i+1};
            end
        end
    end
    
    % IK algorithm starts here
    disp('Solving inverse kinematics...')
    count = 0;                                                              % Counter
    e = 1E6*ones(6,1);                                                      % Start with high initial pose error    de = zeros(6,1);                                                        % Change in error
    de = zeros(6,1);                                                        % Change in error
    q = q0;                                                                 % Current joint state
    dq = zeros(obj.n);                                                      % Step size in joint space
    
    while count < steps
        pose = obj.fk(q,origin);                                              % Compute tool pose at current state
        poseError = pose.error(desired);                                           % Compute pose error at this configuration
  
        J = obj.getJacobian(q);                                             % Compute Jacobian for current configuration
        
        switch method
            case 'inverse'
                [U,S,V] = svd(J);
                invS = zeros(obj.n,6);
                for(i = 1:6)
                    if(S(i,i) > 1e-4)
                        invS(i,i) = 1/S(i,i);
                    end
                end
              
                dq = V*invS*U'*0.5*poseError;
            case 'transpose'
                dq = 0.9*J'*poseError;
            otherwise
                disp("Method incorrectly specified. Please input 'transpose' or 'inverse'.");
        end
        
        if norm(e) < minstep
            disp(['Minimum step size attained in ', num2str(count), ' steps.'])
            break
        end
       
        q = q + dq;                                                         % Update new joint configuration
        
        %Ensure the solution is within joint limits  
        for j = 1:obj.n
            if q(j) > obj.link(j).qlim(2)
                q(j) = obj.link(j).qlim(2);
            elseif q(j) < obj.link(j).qlim(1)
                q(j) = obj.link(j).qlim(1);
            end
        end
        
        count = count + 1;
        
        if animate
            obj.plot3D(q,'workspace',[-0.2 0.6 -0.5 0.5 -0.3 0.7]);
            hold on
                try delete(textHandle)
                end
                textHandle = text(0,0.5,0.5,['Step ', num2str(count)]);
            hold off
                drawnow();
        end

        % Print out information to the console
        if mod(count,round(steps/5)) == 0
            disp(['Step ', num2str(count),' of ',num2str(steps),'...'])
        end
        if count == steps
            disp("Failed to solve inverse kinematics in "+num2str(steps)+" steps. Try altering the initial guess for the joint configuration.")
        end

    end
            
    % Compute the final pose error
    EE = obj.fk(q);                             
    ep = EE.error(desired);
    disp(['Norm of position error is: ', num2str(norm(ep)*1000),' mm.']);
    Q = desired.rot*EE.rot.inverse;
    disp(['Orientation error is: ', num2str(rad2deg(Q.angle)), ' deg.']);
    ret = q;