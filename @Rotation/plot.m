%% plot
% Jonathan Woolfrey
%
% This function plots the axes of a coordinate frame represented by a
% Rotation object.

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

function plot(obj,origin)
    scale = 0.25;
    A = obj.matrix;                                                         % Matrix
    A = A*scale;
    if nargin == 2
        p = origin;
    else
        p = zeros(3,1);
    end
	hold on
        quiver3(p(1),p(2),p(3),A(1,1),A(2,1),A(3,1),'r','LineWidth',1,'MaxHeadSize',1)
        quiver3(p(1),p(2),p(3),A(1,2),A(2,2),A(3,2),'g','LineWidth',1,'MaxHeadSize',1)
        quiver3(p(1),p(2),p(3),A(1,3),A(2,3),A(3,3),'b','LineWidth',1,'MaxHeadSize',1)
	hold off
    
    if isempty(get(groot,'CurrentFigure'))
        axis([p(1)-scale, p(1)+scale, p(2)-scale, p(2)+scale, p(3)-scale, p(3)+scale])
        set(gcf,'Color',[1 1 1])
    end
%             set(gca,'XTick',axis(1:2),'YTick',axis(3:4), 'ZTick',axis(5:6))
    box off
end
