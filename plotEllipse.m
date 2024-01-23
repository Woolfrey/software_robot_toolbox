%% plotEllipse()
% Jon Woolfrey
% August 2020
%
% This function plots an ellipsoid for a positive-definite matrix A (nxn). If
% n = 2, it will draw a 2D ellipse. If n = 3, it will plot a 3D ellipsoid.
%
% The equation:
%
% (p - c)'*A*(p - c) = 1
%
% Defines all points p on the surface of an ellipsoid, characterized by A,
% with centre c.

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

function plotEllipse(A, varargin)

    n = size(A,1);                  % No. of dimensions
    if n ~= size(A,2)
        error("Input matrix must be square.");
    end
    
    % Default setting
    LineWidth = 1;
    LineColor = [0 0 0];
    FaceColor = [0.8 0 0];
    Centre = zeros(n,1);
    Opacity = 0.4;
    % Process options
    for i = 1:length(varargin)
        if ischar(varargin{i})
            switch varargin{i}
                case 'LineWidth'
                    LineWidth = varargin{i+1};
                case 'FaceColor'
                    FaceColor = varargin{i+1};
                case 'Centre'
                    Centre = varargin{i+1};
                case 'LineColor'
                    LineColor = varargin{i+1};
                case 'Opacity'
                    Opacity = varargin{i+1};
            end
        end
    end
        
    % Plot the ellipse    
    switch n
        case 1
            error("Input matrix must be 2x2 or 3x3.");
        case 2
            points = linspace(0,2*pi,50);                   % Points around a unit circle
            unitCircle = [cos(points); sin(points)];        % Create array of x,y coordinates
            if det(A) < 1E-5
                [U, S, V] = svd(A);
                S(2,2) = 1E-10;
                A = U*S*V';
            end
            tf = sqrtm(inv(A));                             % Transform matrix
            ellipse = tf*unitCircle + Centre;               % Transform array, add offset
            plot(ellipse(1,:),ellipse(2,:),'Color',LineColor,'LineWidth',LineWidth);           
        case 3
            [Xa,Ya,Za] = sphere();
            unitSphere = [Xa(:) Ya(:) Za(:)]';              % Points on unit sphere
            tf = sqrtm(inv(A));                             % Transform matrix
            ellipsoid = tf*unitSphere + Centre;            	% Transform unit sphere to ellipsoid, add offset
            
            % Convert the ellipsoid information back to mesh data
            Xb = reshape(ellipsoid(1,:),size(Xa));
            Yb = reshape(ellipsoid(2,:),size(Ya));
            Zb = reshape(ellipsoid(3,:),size(Za));
            
            % Plot the ellipsoid
            mesh(Xb,Yb,Zb,'FaceColor',FaceColor,'EdgeColor',LineColor, ...
                          'FaceAlpha',Opacity, 'EdgeAlpha', Opacity);
            
        otherwise
            error("Cannot plot an ellipse for more than 3 dimensions.");
    end
        
end