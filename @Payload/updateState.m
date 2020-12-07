%% Payload.updateState()
% Jon Woolfrey
% October 2020
%
% This function updates state information about the payload when its parent
% robot is updated.

function updateState(obj,a,adot,omega,FK)
    n = size(omega,2);                              % No. of joints in the manipulator
    obj.pose = FK(n)*obj.TF;                     	% Pose of the object in global frame
    obj.I = obj.pose.rot.rotate(obj.inertia);      	% Rotate inertia tensor to global frame
    for i = 1:n
        obj.r(:,i) = obj.pose.pos - FK(i).pos;    	% Distance from ith joint to object
    end
    
    obj.rdot(:,n) = cross(omega(:,n),obj.r(:,n));   % Time-derivative
    for i = 1:n-1
        j = n-i;                                    % Count backwards
        obj.rdot(:,j) = obj.rdot(:,j+1) + cross(omega(:,j),obj.r(:,j)); % Compute time-derivative recursively
    end

    % Update Jacobian matrix
	n = size(a,2);
    for i = 1:n
        obj.J(1:3,i) = cross(a(:,i),obj.r(:,i));
        obj.J(4:6,i) = a(:,i);
        obj.Jdot(1:3,i) = cross(adot(:,i),obj.r(:,i)) + cross(a(:,i),obj.rdot(:,i));
        obj.Jdot(4:6,i) = adot(:,i);
    end

end