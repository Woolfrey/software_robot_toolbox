%% Payload.plot()
% Jon Woolfrey
% November 2020
%
% This function plots a 3D model of a payload object. Properties such as
% faces, colors, and vertices must be assigned to the object for this
% function to work.

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

function plot(obj)
    if isempty(obj.faces)
        error("Insufficient data to create a 3D model of this object");
    end
    
    % Check to see if model already exists. If so, then simply update the
    % transform.
    modelExists = false;
    fig = gca;
    if isempty(fig.Children)
        light('Position',[-10,0,100]);                      % Create a light source
    else
        for i = 1:length(fig.Children)                      % Cycle through all Children objects of the figure
            if strcmp(fig.Children(i).Tag, obj.name)
            	modelExists = true;                         % Model already exists in the plot
                fig.Children(i).Matrix = obj.pose.matrix;	% Update the pose of the object
            end
        end
    end
    
    % Model doesn't exist, so create it
    if ~modelExists
        object_patch = patch('Faces', obj.faces,            ...
                             'Vertices', obj.vertices,    	...
                             'FaceVertexCData', obj.colors,	...
                             'FaceColor', 'flat',         	...
                             'EdgeColor', 'none');
        payload_tf = hgtransform('Tag',obj.name,'Matrix',obj.pose.matrix);  % Create the transform for the 3D model
        set(object_patch,'Parent',payload_tf);                              % Assign transform object as parent to patch object
    end
end
