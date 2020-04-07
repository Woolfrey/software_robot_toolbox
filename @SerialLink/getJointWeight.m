%% SerialLink.getJointWeight()
% Jonathan Woolfrey
%
% This function returns a weighting matrix for joint limit avoidance of a
% redundant manipulator.
%
% TO DO:
%   - Figure out a proper scaling factor



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

function ret = getJointWeight(obj,pos,vel)
    ret = zeros(obj.n,obj.n);                                            	% Preallocate memory
    for i = 1:obj.n
        s = 5000*(obj.link(i).qlim(2) - obj.link(i).qlim(1))^(-2);        	% Individual joint scalar
        u = obj.link(i).qlim(2) - pos(i);                                   % Distance from upper limit
        v = pos(i) - obj.link(i).qlim(1);                                   % Distance from lower limit
        df = (u^2 - v^2)/(-s*u^2*v^2);                                      % Gradient of penalty function
        fdot = vel(i)*df;                                                   % Time-derivative of penalty function
        if fdot > 0                                                         % If moving toward a joint...
            if abs(df) > 1E10
                ret(i,i) = 1E10;                                            % Ensure the number is not infinite
            else
                ret(i,i) = abs(df);                                         % Add weighting
            end
        end
    end
end

