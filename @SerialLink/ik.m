%% SerialLink.ik()
% Jonathan Woolfrey
%
% This function will solve the inverse kinematics of a serial link
% manipulator for a desired end-effector pose.
%
% By default, this algorithm uses the Jacobian transpose method to maintain
% numerical stabilitity in the solution:
%
%           q(k+1) = q(k) + alpha*J'*e
%
% However, the Newton-Raphson method, which uses the inverse of the
% Jacobian, can be specified as an option.
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
	steps = 3E3;
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
        EE = obj.fk(q,origin);                                              % Compute tool pose at current state
        temp = EE.error(desired);                                           % Compute pose error at this configuration
        if count > 1
            de = temp - e;                                                  % Change in error since previous step
        end     
        
        e = temp;                                                           % Update error
                      
        J = obj.getJacobian(q);                                             % Compute Jacobian for current configuration
        
        switch method
            case 'inverse'
                if obj.n > 6
                    W = obj.getInertia(q);                                  % Use inertia weighting
                else
                    W = eye(obj.n);
                end
                dq = obj.dls(J,1E-1,0.5,W)*(e - 0.75*de);               	% Use pseudoinverse Jacobian
                
            case 'transpose'
                dq = J'*(e-0.4*de);                                         % Use transpose
                
            otherwise
                disp("Method incorrectly specified. Please input 'transpose' or 'inverse'.");
        end
        
        if norm(e) < minstep
            disp(['Minimum step size attained in ', num2str(count), ' steps.'])
            break
        end
        qPrev = q;
        q = q + dq;                                                         % Update new joint configuration
        count = count + 1;
        
        % Ensure joint limits are obeyed in the solution
        for j = 1:obj.n
            if q(j) < obj.link(j).qlim(1)
                q(j) = obj.link(j).qlim(1);
            elseif q(j) > obj.link(j).qlim(2)
                q(j) = obj.link(j).qlim(2);
            end
        end
        
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
            disp('Maximum number of iterations executed.')
        end

    end
            
    % Compute the final pose error
    EE = obj.fk(q);                             
    ep = EE.error(desired);
    disp(['Norm of position error is: ', num2str(norm(ep)*1000),' mm.']);
    Q = desired.rot*EE.rot.inverse;
    disp(['Orientation error is: ', num2str(rad2deg(Q.angle)), ' deg.']);
    ret = q;