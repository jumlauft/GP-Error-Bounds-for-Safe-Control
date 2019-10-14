function [dxdt] = dynRobot(t,x,u,p)
%DYNROBOT dynamics of planar robot
% IN: 
%   t       1 x 1       time
%   x       2*nDof x 1  state, joints: q,qdot
%   ctrl    @fun        (x,t)-> u
%   p              
%    .Cfun   @fun       X -> Coriolos Matrix 
%    .Mfun   @fun       X -> Mass Matrix 
% OUT: 
%   dxdt  E x 1
% Copyright (c) by Jonas Umlauft under BSD License
% Last modified: Jonas Umlauft 04/2019

nDof = size(x,1)/2;
dxdt = zeros(2*nDof,1);
id = 2:2:2*nDof;
nid = 1:2:2*nDof;

dxdt(id) = p.Mfun(x)\(u(t,x)-p.Cfun(x)*x(id));
dxdt(nid) = x(id);
end