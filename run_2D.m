% Copyright (c) by Jonas Umlauft (TUM) under BSD License 
% Last modified: Jonas Umlauft 2019-10
clearvars; clear; close all; clc;rng default; setname = '2D';

%% Set Parameters
disp('Setting Parameters...')

% Basic Parameters
Ntr = 100;    % Number of training points
Tsim = 30;   % Simulation time
Nsim = 200;  % Simulation steps
sn = 0.1;     % Observation noise (std deviation)
E = 2;        % State space dimension

% Initial State /reference for simulation
x0 = [0 0]';
ref = @(t) refGeneral(t,E+1,@(tau) 2*sin(tau));  % circle


% Controller gains
pFeLi.lam = ones(E-1,1);
pFeLi.kc = 1;

% Define Systemdynamics
a = 1; b = 1; c = 0;
pdyn.f = @(x) 1-sin(x(1,:)) + b*sigmf(x(2,:),[a c]);
pdyn.g = @(x) 1;


% GP learning and simulation  parameters
optGPR = {'KernelFunction','ardsquaredexponential','ConstantSigma',true,'Sigma',sn};
odeopt = odeset('RelTol',1e-3,'AbsTol',1e-6);

% Visualization
Nte = 1e4; XteMin = [-6 -4]; XteMax = [4 4];
Ndte = floor(nthroot(Nte,E));  Nte = Ndte^E;
Xte = ndgridj(XteMin, XteMax,Ndte*ones(E,1)) ;
Xte1 = reshape(Xte(1,:),Ndte,Ndte); Xte2 = reshape(Xte(2,:),Ndte,Ndte);
Ntrajplot = 100;

% Lyapunov test
tau = 1e-4;     % Grid distance
delta = 0.01;     % Probability for error bound
deltaL = 0.01;     % Probability for Lipschitz constant

%%  Generate Training Points
disp('Generating Training Points...')
Ntr = floor(nthroot(Ntr,E))^E;
Xtr = ndgridj([0 -3],[3 3],sqrt(Ntr)*ones(E,1));
Ytr = pdyn.f(Xtr) +  sn.*randn(1,Ntr);

%% Learn Model - Optimize Hyperparameters
disp('Learning GP model...')
gprModel = fitrgp(Xtr',Ytr',optGPR{:});
pFeLi.f = @(x) predict(gprModel,x'); pFeLi.g = pdyn.g;
sigfun = @(x) nth_output(2, @predict, gprModel,x');
kfcn = gprModel.Impl.Kernel.makeKernelAsFunctionOfXNXM(gprModel.Impl.ThetaHat);
ls = exp(gprModel.Impl.ThetaHat(1:E));  sf = exp(gprModel.Impl.ThetaHat(end));


%% Test Lyapunov condition
disp('Setup Lyapunov Stability Test...')

Lf =  max(sqrt(sum(gradestj(pdyn.f,Xte).^2,1)));
Lk = norm(sf^2*exp(-0.5)./ls);

k = @(x,xp) sf^2 * exp(-0.5*sum((x-xp).^2./ls.^2,1));
dkdxi = @(x,xp,i)  -(x(i,:)-xp(i,:))./ls(i)^2 .* k(x,xp);
ddkdxidxpi = @(x,xp,i) ls(i)^(-2) * k(x,xp) +  (x(i,:)-xp(i,:))/ls(i)^2 .*dkdxi(x,xp,i);
dddkdxidxpi = @(x,xp,i) -ls(i)^(-2) * dkdxi(x,xp,i) - ls(i)^(-2) .*dkdxi(x,xp,i) ...
    +  (x(i,:)-xp(i,:))/ls(i)^2 .*ddkdxidxpi(x,xp,i);

r = max(pdist(Xte')); Lfs = zeros(E,1);
for e=1:E
    maxk = max(ddkdxidxpi(Xte,Xte,e));
    Lkds = zeros(Nte,1);
    for nte = 1:Nte
       Lkds(nte) = max(dddkdxidxpi(Xte,Xte(:,nte),e));
    end
    Lkd = max(Lkds);  
    Lfs(e) = sqrt(2*log(2*E/deltaL))*maxk + 12*sqrt(6*E)*max(maxk,sqrt(r*Lkd));
end
Lfh =  norm(Lfs);
Lnu = Lk*sqrt(Ntr)*norm(gprModel.Alpha);
omega = sqrt(2*tau*Ntr*Lk*norm(kfcn(Xtr',Xtr')+sn^2*eye(Ntr))*sf^2);
gamma = tau*(Lnu+Lfh) + omega;
beta = log((1+((max(XteMax)-min(XteMin))/tau))^E/delta);

Lyapincr = @(X,r) sqrt(sum((X-r).^2,1))' <= (sqrt(beta).*sigfun(X)+gamma)/(pFeLi.kc*sqrt(pFeLi.lam^2+1));

%% Simulate System with Feedback Linearization and PD Controller
disp('Simulation...')
dyn = @(t,x) dynAffine(t,x,@(t,x) ctrlFeLi(t,x,pFeLi,ref),pdyn);
[T,Xsim] = ode45(dyn,linspace(0,Tsim,Nsim),x0); Xsim= Xsim';
Xd = ref(T);Xd = Xd(1:E,:);
AreaError = zeros(Nsim,1); ihull = cell(Nsim,1);
for nsim=1:Nsim
    ii = find(Lyapincr(Xte,Xd(1:2,nsim)));
    ihull{nsim} = ii(convhull(Xte(:,ii)','simplify',true));
    AreaError(nsim) = polyarea(Xte(1,ihull{nsim}),Xte(2,ihull{nsim}));
end


%% Viualization
disp('Plotting Results...')
vis_2D;
disp('Pau');