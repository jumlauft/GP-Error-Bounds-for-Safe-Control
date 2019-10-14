function out = nth_element(ii, fcn, varargin)
%NTH_OUTPUT returns Nth output value of a function fcn
% In:
%    N      ndx x 1   cell containing indexes for each dimension
%   fcn     fhandle
% Out:
%   out     N-th output of fcn
% Copyright (c) by Jonas Umlauft (TUM) under BSD License 
% Last modified: Jonas Umlauft 10/2018

A = fcn(varargin{:});
out = A(ii{:});
end

