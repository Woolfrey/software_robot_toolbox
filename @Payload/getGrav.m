%% Payload.getGrav()
% Jon Woolfrey
% November 2020
%
% This function computes the joint torques needed to support the payload
% against gravitational acceleration

function ret = getGrav(obj,g)
    if nargin == 1
        ret = obj.mass*obj.J(1:3,:)'*[0;0;9.81];
    else
        ret = obj.mass*obj.J(1:3,:)'*g;
    end
end