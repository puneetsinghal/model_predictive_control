clear;
clc;
close all;
profile on;

%% Initialize

Ts = 0.01;
N = 30;
l = [1.0 0.5 0.2];
finaltheta = [1.0781; 1.0701; 0.993];
%finaltheta = [0.6781; 1.0701; 0.993];
finalthetaRest = findFeasibleConfigurationAnalytical(finaltheta,l);
finaltheta = [finaltheta; finalthetaRest];
finalthetadot = [0;0;0;0;0;0];
qfinal = [finaltheta;finalthetadot]; 

starttheta = [0.6781; 1.0701+0.4; 0.993-0.4];
%starttheta = finaltheta(1:3,1) + [0; 0.1; 0.1];
startthetaRest = findFeasibleConfigurationAnalytical(starttheta,l);
starttheta = [starttheta; startthetaRest];
startthetadot = [0;0;0;0;0;0];
qstart = [starttheta;startthetadot];

x = qstart; % Initial State
xf = qfinal;% Final State                                          
xHist = zeros(12,N);
uHist = zeros(2,N);
xint = x;
%% DirCol

%Initial Optimized Input Values over the trajectory
p0 = zeros(14,N);
p0(1:12,:) = x + (xf-x)*(0:1:(N-1))/(N-1);

%Fmincon Options
options = optimoptions(@fmincon,'StepTolerance',1e-15,'FunctionTolerance',1e-8,'MaxIterations',10000,'MaxFunctionEvaluations',500000,...
                       'DiffMinChange',1e-3,'Display','iter','Algorithm','interior-point');

%Input Bounds
lb_input = -inf;
ub_input = inf;
LB_ = [0    ;0  ;0   ;pi/2;-pi;-pi;-inf;-inf;-inf;-inf;-inf;-inf;lb_input;lb_input];
UB_ = [pi/2 ;pi ;pi/2;pi  ;0  ;0  ;inf ;inf ;inf ;inf ;inf ;inf ;ub_input;ub_input];
LB = LB_;
UB = UB_;
for i = 1:N-1
    LB = [LB ,LB_];
    UB = [UB ,UB_];
end

% for ct = 1:50
%     sprintf('iteration #: %d', ct)
%     xHist(:,ct) = xint;
COSTFUN = @(p) cost(p, xf);
CONSTFUN = @(p) goatConstraintFCNGLDC(p,x,xf,Ts,N,l);
optimal_parameters = fmincon(COSTFUN,p0,[],[],[],[],LB,UB,CONSTFUN,options);
% uHist(:,ct) = optimal_parameters(13:14,1);
% [xint, y] = goatDynamicsDT(xint,uHist(:,ct), Ts, N,l);
% % p0 = zeros(14,N);
% % p0(1:12,:) = xint + (xf-xint)*(0:1:(N-1))/(N-1);
%end
%% Find infeasible points
[~,const] = CONSTFUN(optimal_parameters);
infeasible = find(abs(const)>0.05);

%% Plot Trajectory
plotDC(Ts,optimal_parameters,xf);

%% Visualize Trajectory
visualizeTrajectory(optimal_parameters',l);

%% Cost Function
function J = cost(p,xref)
    Q = ones(12,1);
    Q(1:6,1)=10*Q(1:6,1);
    R = 0.01*zeros(2,1);
    [~,N] = size(p);
    Q = Q*ones(1,N);
    R = R*ones(1,N);
    xref = [xref*ones(1,N);zeros(2,N)];
    J = (p - xref).*[Q;R].*(p - xref);
    J = sum(sum(J));
end

