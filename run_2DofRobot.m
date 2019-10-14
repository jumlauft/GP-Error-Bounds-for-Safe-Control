clearvars; clear; close all; clc;rng default; 
setname = '2DofRobot';

%% Set Parameters
disp('Setting Parameters...')
% Robot kinematrics
pdyn.L1 = 1; pdyn.L2 = 1; pdyn.R1 = pdyn.L1/2; pdyn.R2 = pdyn.L2/2;
pdyn.M1 = 1; pdyn.M2 = 1; pdyn.Iz1 = 1; pdyn.Iz2 = 1;
[pdyn.Mfun,pdyn.Cfun,pdyn.f] = getdyn2Dof(pdyn); pFeLi.Mfun = pdyn.Mfun;
nDof = 2; E = 2*nDof;   % State space dimension
ipos = 1:2:2*nDof;      % Indices for positions
sR = 0.05;              % Size of joints in visualization

Ntr = 1e2;              % Number of training points
XtrMin = -1*ones(1,E);  % Area for training data
XtrMax = 1*ones(1,E);   % Area for training data
Tsim = 10;              % Simulation time
Nsim = 100;             % Simulation steps
sn = 0.1*ones(nDof,1);  % Observation noise (std deviation)

% Lyapunov test
tau = 1e-4;             % Grid distance
delta = 0.01;           % Probability
deltas = ones(nDof,1)*delta/nDof;

% Initial State /reference for simulation
x0 = [0 0 0 0]';
ref{1} = @(t) refGeneral(t,2+1,@(tau) 0.5*sin(tau));  % circle
ref{2} = @(t) refGeneral(t,2+1,@(tau) 1.5*cos(2*tau));  % circle

% Controller gains
pFeLi.lam = ones(nDof,1);
pFeLi.kc = 5*ones(nDof,1);

% GP learning and simulation parameters
optGPR = {'KernelFunction','ardsquaredexponential','ConstantSigma',true,'Sigma',sn};
odeopt = odeset('RelTol',1e-3,'AbsTol',1e-6);

% Test points / State Space
Nte = 1e4;              % Number of training points
XteMin = -pi*ones(1,E); % Area for test data
XteMax = pi*ones(1,E);  % Area for test data

frame2plot = 70;         % Frame to be plotted

%%  Generate Training and Test Data
disp('Generating Training and Test Data...')

Ndte = floor(nthroot(Nte,E));  Nte = Ndte^E;
Xte = ndgridj(XteMin, XteMax,Ndte*ones(E,1)) ;
Ndtepos = floor(nthroot(Nte,nDof));  Ntepos = Ndtepos^nDof; Xtepos = zeros(E,Ntepos);
Xtepos(1:2:2*nDof,:) = ndgridj(XteMin(1:2:2*nDof), XteMax(1:2:2*nDof),Ndtepos*ones(nDof,1));
Xtepos1 = reshape(Xtepos(1,:),Ndtepos,Ndtepos); Xtepos2 = reshape(Xtepos(3,:),Ndtepos,Ndtepos);

Ntr = floor(nthroot(Ntr,E))^E;
Xtr = ndgridj(XtrMin,XtrMax,nthroot(Ntr,E)*ones(E,1));
Ytr = pdyn.f(Xtr) +  sn.*randn(nDof,Ntr);

%% Learn Model - Optimize Hyperparameters
disp('Learning GP model...')
[pFeLi.f,sigfun,gprMdls] = learnGPR(Xtr,Ytr,optGPR{:});

%% Test Lyapunov condition
disp('Setting up Lyapunov Stability Test...')
LyapDecr = cell(nDof,1);
for ndof = 1:nDof
    kfcn = gprMdls{ndof}.Impl.Kernel.makeKernelAsFunctionOfXNXM(gprMdls{ndof}.Impl.ThetaHat);
    ls = exp(gprMdls{ndof}.Impl.ThetaHat(1:E));  sf = exp(gprMdls{ndof}.Impl.ThetaHat(end));
    Lf = max(sqrt(sum(gradestj(@(x) nth_element({ndof,1:size(x,2)},pdyn.f,x),Xte).^2,1)));
    Lk = norm(sf^2*exp(-0.5)./ls);   Lnu = Lk*sqrt(Ntr)*norm(gprMdls{ndof}.Alpha);
    omega = sqrt(2*tau*Ntr*Lk*norm(kfcn(Xtr',Xtr')+sn(ndof)^2*eye(Ntr))*sf^2);
    gamma = tau*(Lnu+Lf) + omega;   beta = log((1+((max(XteMax)-min(XteMin))/tau))^E/deltas(ndof));
    
    LyapDecr{ndof} = @(X,r) sqrt(sum((X-r).^2,1)) >= ...
        (sqrt(beta).*sqrt(nth_element({ndof,1:size(X,2)},sigfun,X))+gamma)/(pFeLi.kc(ndof)*sqrt(pFeLi.lam(ndof)^2+1));
end

%% Simulate System with Feedback Linearization
disp('Simulating Controlled System...')
dyn = @(t,x) dynRobot(t,x,@(t,x) ctrlFeLiRob(t,x,pFeLi,ref),pdyn);
[T,Xsim] = ode45(dyn,linspace(0,Tsim,Nsim),x0); Xsim= Xsim';

%% Simulate System with Feedback Linearization
disp('Evaluating Stability along Trajectory...')
Xd = zeros(E,Nsim);Xdpos = zeros(E,Nsim);
for ndof = 1:nDof
    re = ref{ndof}(T); Xdpos(2*ndof-1,:) = re(1,:); Xd(2*ndof-1:2*ndof,:) = re(1:2,:);
end
iiLyapDecr = zeros(Nsim,Ntepos);    iLyapDecr = true(Ntepos,nDof);
for nsim = 1:Nsim
    for ndof = 1:nDof
        iLyapDecr(:,ndof) = LyapDecr{ndof}(Xtepos,Xdpos(:,nsim));
    end
    iiLyapDecr(nsim,:) = and(iLyapDecr(:,1),iLyapDecr(:,2))';
end

%% Viualization and Saving
disp('Plotting Results and Saving...')
vis_2DofRobot;
disp('Pau');
