%% Payload.getCoriolis()
% Jon Woolfrey
% October 2020
%
% This function gets the Coriolis and centripetal effects of a payload
% object in the joint space of a manipulator.

function ret = getCoriolis(obj)
    ret = obj.mass*obj.J(1:3,:)'*obj.Jdot(1:3,:) ...
        + obj.J(4:6,:)'*obj.I*obj.Jdot(4:6,:) ...
        + obj.J(4:6,:)'*obj.Idot*obj.J(4:6,:);
end