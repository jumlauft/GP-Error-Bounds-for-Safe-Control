function [Mfun,Cfun,f] = getdyn2Dof(p)
%GETDYN2DOF Build dynamics of 2 degree of freedom robot
% In:
%   p.
%       L1      1 x 1   Length of link 1
%       L2      1 x 1   Length of link 2
%       M1      1 x 1   Mass of link 1
%       M2      1 x 1   Mass of link 2
%       Iz1     1 x 1   Inertia in z of link 1
%       Iz2     1 x 1   Inertia in z of link 2
%       R1      1 x 1   Center of Mass of link 1
%       R2      1 x 1   Center of Mass of link 2
% Out:
%   Mfun       	@fun       X -> Coriolos Matrix 
%   Mfun       	@fun       X -> Mass Matrix 
%   f           @fun       uncontrolled dynamics
% see http://www.cds.caltech.edu/~murray/books/MLS/pdf/mls94-manipdyn_v1_2.pdf
% Copyright (c) by Jonas Umlauft (TUM) under BSD License 
% Last modified: Jonas Umlauft 04/2019

a = p.Iz1 + p.Iz2 + p.M1*p.R1^2 + p.M2*(p.L1^2+p.R2^2);
b = p.M2*p.L1*p.R2;
c = p.Iz2 + p.M2*p.R2^2;

Mfun = @(x) [a+2*b*cos(x(3)) c+b*cos(x(3));
    c+b*cos(x(3)) c];
Cfun = @(x) [-b*sin(x(3))*x(4)  -b*sin(x(3))*(x(2)+x(4));
    b*sin(x(3))*x(2) 0];

f = @(x) fun(x,Mfun,Cfun);

end

function f = fun(x,Mfun,Cfun)
N = size(x,2); nDof = size(x,1)/2;id = 2:2:2*nDof;
f = zeros(nDof,N);
for n=1:N
    f(:,n) = -Mfun(x(:,n))\(Cfun(x(:,n))*x(id,n));
end
end
