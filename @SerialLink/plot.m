%% Plot a simple model of the manipulator
% Jonathan Woolfrey
%
% This function plots a simple geometric model of a serial link
% manipulator.
%
% To Do:
%   - Find a way to declare arrays of handles for surfl and hgtransform
%   - Only delete transforms associated with the robot model
%   - Merge this method with plot3D



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

function plot(obj,q,varargin)
    if nargin == 1                                                          % Use current state
        q = obj.q;                                                          % Get the current robot state
    end
    
    [~,FK] = obj.fk(q,obj.base);                                            % Compute the forward kinematics

    % Default options
    axes = false;
	workspace = [-1 1 -1 1 -0.5 1.5];      
    
    % Process options
    for i = 1:length(varargin)
        if ischar(varargin{i})
            option = varargin{i};
            switch option
                case 'axes'
                    axes = true;
                case 'workspace'
                    workspace = varargin{i+1};
                otherwise
                    error("Option(s) for SerialLink.plot() incorrectly specified.");
            end
        end
    end           
    
    
    % Check to see if a figure exists, and delete any transform objects.                                                                  
    fig = gca;                                                              % Assign graphics handle
    if isempty(fig.Children)                                                % Nothing plotted yet
        light('Position',[100,10,100]);                                      % Create a light source
    else
        % Run through and delete the hggroup containing the robot model
        for i = 1:length(fig.Children)
            if strcmp(fig.Children(i).Type,'hggroup') && strcmp(fig.Children(i).Tag, 'lol')
                delete(fig.Children(i))                                 
                break                                                       
            end
        end
    end
	hg = hggroup('Tag','lol');                                              % Create a new hg group
    
    
    % Loop through every link and draw them
    for i = 1:obj.n                                                        
        p = obj.link(i).getPose(0).pos;                                     % Get position vector in local frame
    
        % Create a cylinder for the joint
        if obj.link(i).isrevolute
            N = 30;                                                         % No. of faces
        else
            N = 4;                                                          % 4 faces for a rectangular prism
        end
        [x,y,z] = cylinder([0,0.07,0.07,0],N);                             	% Generate surface data
        z([1,2],:) = -0.055;                                                % Origin
        z([3,4],:) = 0.055;                                               	% End
        hold on
            h(1,i) = surfl(x,y,z);                                       	% Create surface
        hold off

        % Create a cylinder for the link in the z direction
        if obj.link(i).isrevolute                                        	% Revolute joint
            N = 30;                                                      	% No. of faces
        else
            N = 4;                                                        	% 4 faces for a rectangular prism
        end
        [x,y,z] = cylinder([0,0.05,0.05,0],N);                             	% Generate surface data
        z([1,2],:) = 0;                                                   	% Origin
        if obj.link(i).isrevolute                                        	% Revolute joint
            z([3,4],:) = p(3);
        else
            z([3,4],:) = p(3) + q(i);                                     	% Add displacement for prismatic joint
        end
        hold on
            h(2,i) = surfl(x,y,z);                                      	% Create surface
        hold off
        
        % Create a cylinder for the link in the x-direction
        [x,y,z] = cylinder([0,0.05,0.05,0],30);                             % Generate surface
        z([1,2],:) = 0;                                                     % Origin
        z([3,4],:) = obj.link(i).a;                                         % Translation in x-direction                                            
        hold on
            h(3,i) = surfl(x,y,z);
        hold off
        
        % Set the properties for display
        for j = 1:3
            if j == 1
                h(j,i).FaceColor = [0.1 0.1 0.1];                         	% Joint color
            else
                h(j,i).FaceColor = [0.7 0.7 0.7];                        	% Link color
            end
            h(j,i).MeshStyle = 'none';                                      % No edges
            h(j,i).FaceLighting = 'flat';                                       
            h(j,i).BackFaceLighting = 'lit';
            h(j,i).EdgeLighting = 'flat';
        end

        if i == 1
            TF(i) = hgtransform('Tag','lol','Matrix',obj.base.matrix,'Parent',hg); % First link uses base transform
        else
            TF(i) = hgtransform('Tag','lol','Matrix',FK(i-1).matrix,'Parent',hg); % Link i transformed by i-1
        end
        set(h(1,i),'Parent',TF(i));                                         % Transform the joint cylinder
        set(h(2,i),'Parent',TF(i));                                         % Transform the link cylinder
        M = [0 0 -1 0
             0 1 0 0
             1 0 0 0
             0 0 0 1];
        set(h(3,i),'Parent',hgtransform('Tag','lol','Matrix',FK(i).matrix*M,'Parent',hg));
        
        if axes
            if i == 1
                obj.base.plot
            else
                FK(i-1).plot
            end
        end
    end
    
    % Set figure properties
    box on                                                                  % Put borders around the figure
    axis(workspace);                                                        % Set the axes
    view(-30,30);                                                           % Set the view
    rotate3d on;                                                            % Allow 3D rotation
    drawnow();                                                              % Now draw the plot
end