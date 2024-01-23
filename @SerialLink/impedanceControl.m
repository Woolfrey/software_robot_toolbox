%% SerialLink.impedanceControl
% Jon Woolfrey
% April 2021
%
% This function computes the joint torques required to perform
% impedance/admittance control.
%
% Inputs:
% - acc (6x1):          Desired Cartesian acceleration
% - Ad  (6x6):          Desired Cartesian inertia
% - vel (6x1):          Desired Cartesian velocity
% - D   (6x6):          Cartesian damping matrix
% - pos (Pose object):  Desired end-effector pose
% - K   (6x6):          Cartesian stiffness matrix
% - w   (6x1):          Wrench to be applied by the end-effector
% - tau_r (nx1):        Redundant joint torques

function ret = impedanceControl(obj,acc,Ad,vel,D,pos,K,w,tau_r)

    J = obj.getJacobian();              % Get Jacobian at current joint state
    Jdot = obj.getJdot();               % Time-derivative at current joint state
    A = inv(J/obj.M*J');                % Actual Cartesian inertia
    NT = eye(obj.n,obj.n) - J'*A*J'/M;  % Null space projection matrix (transpose)
    
    e = obj.tool.error(pos);            % Pose error
    edot = vel - J*obj.qdot;            % Velocity error
    
    wd = A*(acc + Ad\(D*edot + K*e) - Jdot*qdot);   % Dynamic wrench for trajectory tracking
    
    if norm(w) == 0                     % No wrench to be applied 
        tau = J'*wd + obj.C*obj.qdot + NT*tau_r;
    else                                
        w_hat = w/norm(w);              % Unit vector in direction of w
        P = eye(6) - w_hat*w_hat';      % Null space of wrench
        wd = P*wd;                      % Project trajectory tracking task on to null space
        tau = J'*(wd + w) + obj.C*obj.qdot + NT*tau_r;
    end
    ret = tau;
end