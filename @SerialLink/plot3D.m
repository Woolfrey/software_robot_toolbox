%% SerialLink.plot3D
% Jonathan Woolfrey
%
% This functions plots a 3D model of a serial link manipulator.
%
% TO DO:
%   - Merge this method with SerialLink.plot



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

function plot3D(obj,q,varargin)

    for i = 1:obj.n
        if isempty(obj.link(i).faces)
            error("Insufficient data to create a 3D model of this robot.");
            break
        end
    end
    
    if nargin == 1
        q = obj.q;                      % Plot using the current joint state
    end
    
    [~,FK] = obj.fk(q,obj.base);        % Get forward kinematics chain
    
    % Default options
    axes = false;
	workspace = [-0.2 1.0 -0.6 0.6 -0.1 1.1];                                      
    perspective = [-30 30];
    toolFrame = true;
    
    % Process optional inputs
    for i = 1:length(varargin)
        if ischar(varargin{i})
            option = varargin{i};
            switch option
                case 'axes'
                    axes = true;
                case 'workspace'
                    workspace = varargin{i+1};
                case 'view'
                    perspective = varargin{i+1};
                case 'noToolFrame'
                    toolFrame = false;
                otherwise
                    error("Option(s) for SerialLink.plot3D() incorrectly specified.");
            end
        end
    end
    
    % Check to see if a robot model already exists, and update transforms
    % accordingly
    modelExists = false;
    fig = gca;
    if isempty(fig.Children)
        light('Position',[-10,0,100]);                                      % Create a light source
    else
        for i = 1:length(fig.Children)
            if strcmp(fig.Children(i).Type, 'hggroup') && strcmp(fig.Children(i).Tag,obj.name)
                modelExists = true;
                m = length(fig.Children(i).Children);                   % m = obj.n + 1
                fig.Children(i).Children(m).Matrix = obj.base.matrix;   % Update base location
                for j = 1:m-1
                    index = m-j;                                         % Order of hgtransforms is backwards
                    fig.Children(i).Children(index).Matrix = FK(j).matrix;	% Update jth link
                end
                break
            end
        end
    end

    % Model doesn't yet exist, so create patch objects and hgtransforms for
    % base and each link
    if ~modelExists
        hg = hggroup('Tag',obj.name);

        % Plot the base
        base_patch = patch('Faces', obj.basefaces,    	...
                     'Vertices', obj.basevertices,    	...
                     'FaceVertexCData', obj.basecolors,	...
                     'FaceColor', 'flat',              	...
                     'EdgeColor', 'none');
        base_tf = hgtransform('Tag',obj.name+"_base",'Matrix',obj.base.matrix,'Parent',hg);	% Create a transform for the base
        set(base_patch,'Parent',base_tf);                                               % Set the transform
        
        % Plot the links
        link_patch = nan(obj.n,1);                                                      	
        link_tf = nan(obj.n,1);                                                      
        for i = 1:obj.n
           link_patch(i) =  patch('Faces',obj.link(i).faces,          	...
                                  'Vertices',obj.link(i).vertices,      ...
                                  'FaceVertexCData',obj.link(i).colors, ...
                                  'FaceColor','flat',                   ...
                                  'EdgeColor','none');

            link_tf(i) = hgtransform('Tag',obj.name+"_link"+num2str(i),'Matrix',FK(i).matrix,'Parent',hg); % Create a transform for the link
            set(link_patch(i),'Parent',link_tf(i));                                       	
        end
    end

%%
%     if isempty(obj.link(1).faces)
%         error('No 3D information specified for this robot.');
%     end
%     if nargin == 1                                                          % No inputs
%         q = obj.q;                                                          % Use current state
%     end
%     
%     [~,FK] = obj.fk(q,obj.base);                                            % Compute the forward kinematics
%     
%     % Default options
%     axes = false;
% 	workspace = [-0.3 0.7 -0.5 0.5 -0.2 0.8];                                      
%     perspective = [-30 30];
%     toolFrame = true;
%     
%     % Process options
%     for i = 1:length(varargin)
%         if ischar(varargin{i})
%             option = varargin{i};
%             switch option
%                 case 'axes'
%                     axes = true;
%                 case 'workspace'
%                     workspace = varargin{i+1};
%                 case 'view'
%                     perspective = varargin{i+1};
%                 case 'noToolFrame'
%                     toolFrame = false;
%                 otherwise
%                     error("Option(s) for SerialLink.plot3D() incorrectly specified.");
%             end
%         end
%     end
%        
%     % Check to see if a figure exists, and delete any transform objects.                                                                  
%     fig = gca;                                                              % Assign graphics handle
%     if isempty(fig.Children)                                                % Nothing plotted yet
%         light('Position',[-10,0,100]);                                      % Create a light source
%     else
%         % Run through and delete the hggroup containing the robot model
%         for i = 1:length(fig.Children)
%             if strcmp(fig.Children(i).Type,'hggroup') && strcmp(fig.Children(i).Tag, 'lol')
%                 delete(fig.Children(i))                                 
%                 break                                                       
%             end
%         end
%     end
%     
%     hg = hggroup('Tag','lol');                                              % Create a new hg group
%     
%     % Plot the base
%     blah = patch(   'Faces', obj.basefaces,             ...
%                     'Vertices', obj.basevertices,       ...
%                     'FaceVertexCData', obj.basecolors,  ...
%                     'FaceColor', 'flat',                ...
%                     'EdgeColor', 'none');
%     stuff = hgtransform('Tag','lol','Matrix',obj.base.matrix,'Parent',hg);	% Create a transform for the base
%     set(blah,'Parent',stuff);                                               % Set the transform
%     
%     % Plot the links
%     h = nan(obj.n,1);                                                      	
%     TF = nan(obj.n,1);                                                      
%     for i = 1:obj.n
%        h(i) =  patch(   'Faces',obj.link(i).faces,                  ...
%                         'Vertices',obj.link(i).vertices,            ...
%                         'FaceVertexCData',obj.link(i).colors,    	...
%                         'FaceColor','flat',                     	...
%                         'EdgeColor','none');
%                     
%         TF(i) = hgtransform('Tag','lol','Matrix',FK(i).matrix,'Parent',hg); % Create a transform for the link
%         set(h(i),'Parent',TF(i));                                       	
%     end
%%

    % Plot axes for each link
    if axes == true
        hold on
        obj.base.plot;
        for j = 1:obj.n
           hold on
           FK(j).plot;
           hold off
        end
    end
    
%     if toolFrame
%         hold on
%             FK(obj.n).plot;
%         hold off
%     end
    
    % Set figure properties
    box on                                                                  % Put borders around the figure
    axis equal
    axis(workspace);                                                        % Set the axes
    view(perspective(1),perspective(2));                                    % Set the view
    rotate3d on;                                                            % Allow 3D rotation
    set(gcf,'Color',[1 1 1]);
    xlabel('X (m)')
    ylabel('Y (m)')
    zlabel('Z (m)')
    drawnow();                                                              % Now draw the plot
end