%% rot2quat
% Jonathan Woolfrey
%
% This function converts a rotation matrix defined in SO(3) to a quaternion
% in H.
%
% Inputs:
%   - Rotation matrix (3x3)
%
% Output:
%   - Quaternion (4x1)


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

function quat = rot2quat(R)
    if size(R,1) ~= 3 && size(R,2) ~= 3
        error("Expected a 3x3 matrix as an input.")
        return
    end
    R44 = trace(R);

    Rii = max([R(1,1) R(2,2) R(3,3) R44]);                       	% Find the maximum value


    ci = (1 + 2*Rii - R44)^0.5;                                  	% Calculate the scalar
    switch Rii                                                      % Find the other scalars
        case R(1,1)
            n = (R(3,2) - R(2,3))/ci;
            e2 = (R(2,1) + R(1,2))/ci;
            e3 = (R(1,3) + R(3,1))/ci;
            quat = 0.5*[n; ci; e2; e3];
%                     c2 = (R(2,1) + R(1,2))/ci;
%                     c3 = (R(1,3) + R(3,1))/ci;
%                     c4 = (R(3,2) - R(2,3))/ci;
%                     quat = 0.5*[ci c2 c3 c4]';

        case R(2,2)
            n = (R(1,3) - R(3,1))/ci;
            e1 = (R(2,1) + R(1,2))/ci;
            e3 = (R(3,2) + R(2,3))/ci;
            quat = 0.5*[n; e1; ci; e3];
%                     c1 = (R(2,1) + R(1,2))/ci;
%                     c3 = (R(3,2) + R(2,3))/ci;
%                     c4 = (R(1,3) - R(3,1))/ci;
%                     quat = 0.5*[c1 ci c3 c4]';

        case R(3,3)
            n = (R(2,1) - R(1,2))/ci;
            e1 = (R(1,3) + R(3,1))/ci;
            e2 = (R(3,2) + R(2,3))/ci;
            quat = 0.5*[n; e1; e2; ci];
%                     c1 = (R(1,3) + R(3,1))/ci;
%                     c2 = (R(3,2) + R(2,3))/ci;
%                     c4 = (R(2,1) - R(1,2))/ci;
%                     quat = 0.5*[c1 c2 ci c4]';

        case R44
            e1 = (R(3,2) - R(2,3))/ci;
            e2 = (R(1,3) - R(3,1))/ci;
            e3 = (R(2,1) - R(1,2))/ci;
            quat = 0.5*[ci; e1; e2; e3];
%                     c1 = (R(3,2) - R(2,3))/ci;
%                     c2 = (R(1,3) - R(3,1))/ci;
%                     c3 = (R(2,1) - R(1,2))/ci;
% %                     quat = 0.5*[c1 c2 c3 ci]';
        otherwise
            quat = [1 0 0 0]';
    end
end