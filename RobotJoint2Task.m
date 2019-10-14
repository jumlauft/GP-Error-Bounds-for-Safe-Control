function [Xj2,Xj1] = RobotJoint2Task(theta,p)
%RobotJoint2Task Transforms from joint to task coordinates
% In:
%   theta       2 x N  lower bounds of grid for each dimension separately
%   p.
%       L1      1 x 1   Length of link 1
%       L2      1 x 1   Length of link 2
% Out:
%   Xj2       	2 x N   Position end of link 2 in task space
%   Xj1       	2 x N   Position end of link 1 in task space
%
% Copyright (c) by Jonas Umlauft under BSD License
% Last modified: Jonas Umlauft 04/2019

Xj1(1,:) = p.L1*cos(theta(1,:));
Xj1(2,:) = p.L1*sin(theta(1,:));

Xj2(1,:) = Xj1(1,:) + p.L2*cos(theta(1,:)+theta(2,:));
Xj2(2,:) = Xj1(2,:) + p.L2*sin(theta(1,:)+theta(2,:));

end

