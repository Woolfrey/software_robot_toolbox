%% SerialLink.jointControl()
% Jon Woolfrey
% October 2020
%
% This function computes the joint torques required to drive the
% manipulator joints to a certain position, velocity, acceleration
%
% Inputs:
% - acc (nx1)       Desired joint accelerations
% - vel (nx1)       Desired joint velocities
% - Kd (1x1)        Gain on velocity error
% - pos (nx1)       Desired joint positions
% - Kp (1x1)        Gain on position error
%
% Outputs:
% - tau (nx1)       Joint torques needed to achieve desired joint state

function ret = jointControl(obj,acc,vel,Kd,pos,Kp)
    ret = obj.invDynamics(acc + Kd*(obj.qdot - vel) + Kp*(obj.q - pos));
end