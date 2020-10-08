%% Rotation.quat2rot()
% Jon Woolfrey
% June 2020
%
% This function takes a quaternion (4x1 vector) and converts it in to a 3x3
% rotation matrix.
%
% Inputs:
% - q             A unit vector representing a quaternions (4x1)
%
% Outputs:
% - Rotation matrix (3x3)
function ret = quat2rot(q)
    ret = zeros(3,3);
    ret(1,1) = q(1)^2 + q(2)^2 - q(3)^2 - q(4)^2;
    ret(1,2) = 2*(q(2)*q(3) - q(1)*q(4));
    ret(1,3) = 2*(q(2)*q(4) + q(1)*q(3));
    ret(2,1) = 2*(q(2)*q(3) + q(1)*q(4));
    ret(2,2) = q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2;
    ret(2,3) = 2*(q(3)*q(4) - q(1)*q(2));
    ret(3,1) = 2*(q(2)*q(4) - q(1)*q(3));
    ret(3,2) = 2*(q(3)*q(4) + q(1)*q(2));
    ret(3,3) = q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2;
end