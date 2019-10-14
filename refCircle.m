function reffun = refCircle(xS,xE,tS)
%REFCIRCLE Circulating reference function
%   returns function handle which provides 2 derivatives of a reference
%   which is circle passing through xS at time tS and later through xE
% In:
%    xS      E x 1      starting point of reference trajectory
%    xE      E x 1      point on the circle
%    tS      1 x 1      starting time 
% Out:
%    reffun  fhandle    N x 1 --> E+1 x N

% Copyright (c) by Jonas Umlauft under BSD License 
% Author: Jonas Umlauft, Last modified: 04/2018
%{
clear;  clc; close all; addpath(genpath('mtools')); 
E = 2; xS = randn(E,1);xE = randn(E,1);tS = rand; 
reffun = refCircle(xS,xE,tS);  T = linspace(tS,tS+1.9*pi);
refs = reffun(T);figure; hold on; xlabel('x1'); ylabel('x2'); zlabel('x3');
plot3(refs(1,:),refs(2,:),refs(3,:),'r');
plot3(xS(1),xS(2),0,'r*');text(xS(1),xS(2),0,'x_S');
plot3(xE(1),xE(2),0,'r*');text(xE(1),xE(2),0,'x_E');
%}
E = size(xS,1);
if ~isequal([2 1],size(xS))|| ~isequal(size(xS),size(xE)) || ~isscalar(tS)
    error('wrong input dimension'); 
end
xcc = (sum(xS.^2) - sum(xE.^2))/(2*xS(1) - 2*xE(1));
radius =  abs(sum((xS-xE).^2)*((xS(1)-xE(1))^2 + (xS(2)+ xE(2))^2))^(1/2)/(2*(xS(1) - xE(1)));
angle1 = atan2(xS(2),xS(1)-xcc);
reffun = @(t) [xcc + radius*cos(angle1+(tS-t) + pi*(xS(1)<xE(1)));
                radius*sin(angle1+(tS-t) + pi*(xS(1)<xE(1)));
                -radius*cos(angle1+(tS-t) + pi*(xS(1)<xE(1)));];% cw

%  reffun = @(t) refGeneral(t,E+1,reffun);

end



% a =  (-((x1(1)*x2(2) + x1(2)*x2(1))*(x1(1)*x2(2) - x1(2)*x2(1)))/((x1(2) + x2(2))*(x1(2) - x2(2))))^(1/2);
% b =  (((x1(1)*x2(2) + x1(2)*x2(1))*(x1(1)*x2(2) - x1(2)*x2(1)))/((x1(1) + x2(1))*(x1(1) - x2(1))))^(1/2);


% 
% syms a b x11 x12 x21 x22
% % % eqn = @(a,b) [(x1(1)/a)^2 + (x1(2)/b)^2 - 1;
% % %                 (x2(1)/a)^2 + (x2(2)/b)^2 - 1];
% eqn = @(a,b) [(x11/a)^2 + (x12/b)^2 - 1;
%                 (x21/a)^2 + (x22/b)^2 - 1];
% 
% sol = solve(eqn,[a b])
% plot(a*cos(t),b*sin(t));  
% syms x r x11 x12 x21 x22
% eqn = @(x,r) [(xS(1)-x)^2 + xS(2)^2 - r^2;
%     (xE(1)-x)^2 + xE(2)^2 - r^2; ];
% eqn = @(x,r) [(x11-x)^2 + x12^2 - r^2;
%     (x21-x)^2 + x22^2 - r^2; ];
% sol = solve(eqn,[x r]);
% %             m = (xE(2)-xS(2))/(xE(1)-xS(1)); b = xS(2)- xS(1)*m;
%                         reft = @(t) exp(m*(t))-b/m;
%             xcc = (xS(1)^2 + xS(2)^2 - xE(1)^2 - xE(2)^2)/(2*xS(1) - 2*xE(1));
%             radius =  abs((xS(1)^2 - 2*xS(1)*xE(1) + xS(2)^2 - 2*xS(2)*xE(2) + xE(1)^2 + xE(2)^2)*(xS(1)^2 - 2*xS(1)*xE(1) + xS(2)^2 + 2*xS(2)*xE(2) + xE(1)^2 + xE(2)^2))^(1/2)/(2*(xS(1) - xE(1)));
%             angle1 = atan2(xS(2),xS(1)-xcc);
% tS=1; 
%             reft = @(t) xcc + radius*cos(angle1+(tS-t) + pi*(xS(1)<xE(1)));%.*(angle2-angle1)./(t1-t2)
         
