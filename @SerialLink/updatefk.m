%% SerialLink.updateFK()
% Jon Woolfrey
% June 2020
%
% This function updates the forward kinematics chain when the joint
% state of the SerialLink object is updated. This is done to reduce
% computational time, compared to the SerialLink.fk() function.
%
% TO DO:
%   - Add tool transform to the end
function updatefk(obj)
    
    obj.fkchain(1).pos = obj.base.transform(obj.link(1).pose.pos);
    obj.fkchain(1).rot.quat = Rotation.multiplyQuaternion(obj.base.rot.quat,...
                                                          obj.link(1).pose.rot.quat);
    for j = 2:obj.n
        obj.fkchain(j).pos = obj.fkchain(j-1).transform(obj.link(j).pose.pos);
        obj.fkchain(j).rot.quat = Rotation.multiplyQuaternion(obj.fkchain(j-1).rot.quat, ...
                                    obj.link(j).pose.rot.quat);
    end

     % Need to add tool transform here %
end