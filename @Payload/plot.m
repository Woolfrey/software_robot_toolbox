%% Payload.plot()
% Jon Woolfrey
% November 2020
%
% This function plots a 3D model of a payload object.

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
