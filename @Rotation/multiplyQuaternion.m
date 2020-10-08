%% Rotation.multiplyQuaternion
% Jon Woolfrey
% June 2020
%
% This function multiplies two quaternions without creating an object as a
% return variable. This is done to reduce computational cost, primarily in
% the forward kinematics.

function multiplyQuaternion(obj,a,b)       
    obj.quat(1) = a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4);
    obj.quat(2:4) = a(1)*b(2:4) + b(1)*a(2:4) + cross(a(2:4),b(2:4));
      
    if obj.quat(1) < 0
        obj.quat = -1*obj.quat;
    end

end