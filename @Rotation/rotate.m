%% Rotation.rotate()
% Jonathan Woolfrey
%
% This function will rotate a vector (3x1) or matrix (3x3) by the given
% representation of this Rotation object. For a rotation R in SO(3), a
% vector is rotated as:
%
%               v' = R*v,
%
% and a matrix M (3x3) is rotated via:
%
%               M' = R*M*R'.
%
%   Inputs:
%       - A 3x1 vector, OR
%       - A 3x3 matrix
%
%   Output:
%       - The rotated object


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

function ret = rotate(obj,input)
    if size(input,1) ~= 3                                                  	% Check to see if input dimensions are correct
        error("Expecting a 3x1 vector, or a 3x3 matrix.")                   % Inform user
    end
    R = obj.matrix;                                                         % Get this rotation as SO(3)
    if size(input,2) == 1                                                   % Rotate a vector
        ret = R*input;                                          
    elseif size(input,2) == 3                                               % Rotate a matrix
        ret = R*input*R';
    else
        error("Expecting either a 3x1 vector, or a 3x3 matrix.");
    end
end