function u = ctrlFeLiRob(t,x,p,ref)
%CTRLFELI Feedback Linearizating Controller
% IN:
%   t       1 x 1   current time
%   x       E x N   current state
%   p               Parameter struct
%     .f    @fun    Estimate for f
%     .g    @fun    Estimate for g
%     .kc   1 x 1   controller gain
%     .lam  E-1 x 1 filter coefficient (must be Hurwitz)
%   ref     @fun    reference trajectory
% OUT:
%   u  1 x 1
% E: state space dimension
% Copyright (c) by Jonas Umlauft under BSD License
% Last modified: Jonas Umlauft 10/2018

nDof = size(x,1)/2;
nu = zeros(nDof,1);
for ndof = 1:nDof
    xd = ref{ndof}(t);
    
    e = x(2*ndof-1:2*ndof,:) - xd(1:2);
    
    r = [p.lam(ndof) 1]*e;
    
    nu(ndof) = -p.kc(1)*r -  p.lam(ndof)*e(2) + xd(3);
    
end
u = p.Mfun(x)*(-p.f(x) + nu);


end