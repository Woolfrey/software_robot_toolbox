%% Payload.getInertia()
% Jon Woolfrey
% October 2020
%
% This function gets the inertia of a payload object mapped to the joint
% space of a serial link manipulator.

function ret = getInertia(obj)
    ret = obj.mass*obj.J(1:3,:)'*obj.J(1:3,:) ...
        + obj.J(4:6,:)'*obj.I*obj.J(4:6,:);
end