clear;
clc;
close all;
profile on
addpath('lib')

%Sample Time
Ts = 0.001;

%Prediction Horizon
N = 10;
link_length = [1.0 0.5 0.2];

%Initial State
q20 = [1.0781, 1.0701, 0.9933]';
q21 = findFeasibleConfigurationAnalytical(q20,link_length);
q2 = [q20;q21];
dq2 = [0, 0, 0, 0, 0, 0]';

x = [q2; dq2] + [0.012; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

%Final State
xf = [q2; dq2];

%Initial Optimized Input Values over the trajectory
p0 = zeros(14,N);
p0(1:12,:) = x + (xf-x)*(0:1:(N-1))/(N-1);

%Fmincon Options
options = optimoptions(@fmincon,'StepTolerance',1e-15,'TolFun',1e-8,'MaxIter',10000,'MaxFunEvals',500000,...
                       'DiffMinChange',1e-3,'Display','iter','Algorithm','sqp');

%Input Bounds
lb_input = -100;
ub_input = 100;
LB_ = [0    ;0  ;0   ;pi/2;-pi;-pi;-inf;-inf;-inf;-inf;-inf;-inf;lb_input;lb_input];
UB_ = [pi/2 ;pi ;pi/2;pi  ;0  ;0  ;inf ;inf ;inf ;inf ;inf ;inf ;ub_input;ub_input];

LB = LB_;
UB = UB_;

for i = 1:N-1
    LB = [LB ,LB_];
    UB = [UB ,UB_];
end

COSTFUN = @(p) cost(p, x, xf, N, Ts, link_length);
CONSTFUN = @(p) goatConstraintFCNDC(p,x,xf,Ts,N,link_length);
optimal_parameters = fmincon(COSTFUN,p0,[],[],[],[],LB,UB,CONSTFUN,options);


%% Plot
plotDC(Ts,optimal_parameters,xf);

%% Cost Function
function J = cost(p, xk, xref, N, Ts, link_length)
    Q = eye(12);
    Q(1:6,1:6)=10*Q(1:6,1:6);
    R = 0.01*eye(2);
    J = 0;
    xk1 = xk;
    for i = 1:N
        uk = p(7:8,i);
        xk1 = goatDynamicsDT(xk1, uk, Ts, link_length);
        J = J + (xk1-xref)'*Q*(xk1-xref) + uk'*R*uk;
    end
end