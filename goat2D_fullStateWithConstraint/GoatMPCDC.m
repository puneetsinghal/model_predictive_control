clear;
clc;
close all;

%% Initialize

Ts = 0.001;
N = 100;
l = [1.0 0.5 0.2];
finaltheta = [1.0781; 1.0701; 0.993];
finalthetaRest = findFeasibleConfigurationAnalytical(finaltheta,l);
finaltheta = [finaltheta; finalthetaRest];
finalthetadot = [0;0;0;0;0;0];
qfinal = [finaltheta;finalthetadot]; 

starttheta = finaltheta(1:3,1) + [0.01; 0; 0];
startthetaRest = findFeasibleConfigurationAnalytical(starttheta,l);
starttheta = [starttheta; startthetaRest];
startthetadot = [0;0;0;0;0;0];
qstart = [starttheta;startthetadot];

x = qstart; % Initial State
xf = qfinal;% Final State                                          

%% DirCol

%Initial Optimized Input Values over the trajectory
p0 = zeros(14,N);
p0(1:12,:) = x + (xf-x)*(0:1:(N-1))/(N-1);
%Fmincon Options
options = optimoptions(@fmincon,'StepTolerance',1e-15,'FunctionTolerance',1e-8,'MaxIterations',10000,'MaxFunctionEvaluations',500000,...
                       'DiffMinChange',1e-3,'Display','iter','Algorithm','sqp');

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

COSTFUN = @(p) cost(p, xf);
CONSTFUN = @(p) goatConstraintFCNDC(p,x,xf,Ts,N,l);
optimal_parameters = fmincon(COSTFUN,p0,[],[],[],[],LB,UB,CONSTFUN,options);

%% Find infeasible points
[~,const] = CONSTFUN(optimal_parameters);
infeasible = find(abs(const)>0.05);

%% Plot Trajectory
% plotDC(Ts,optimal_parameters,xf);

%% Visualize Trajectory
% visualizeTrajectory(optimal_parameters',l);

%% Cost Function
function J = cost(p,xref)
    Q = eye(12);
    Q(1:6,1:6)=10*Q(1:6,1:6);
    R = 0.01*eye(2);
    J = 0;
    [~,N] = size(p);
    for i = 1:N
        uk = p(13:14,i);
        xk = p(1:12,i);
        J = J + (xk)'*Q*(xk) + uk'*R*uk;
    end
end

