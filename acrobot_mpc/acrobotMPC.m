clear;
clc;
% addpath('lib')
% addpath('functions')
profile on

params.m1 = 1;
params.m2 = 1;
params.l1 = 1;
params.l2 = 1; 
params.g = 9.81;
params.I1 = 0;
params.I2 = 0;
%Sample Time
params.Ts = 0.01;

profile viewer
%Prediction Horizon
params.N = 20;
params.duration = 2;
%Initial State
theta0 = [pi-0.5; pi-0.1; 0; 0];
x = theta0;
%Final State
xf = [pi; pi; 0; 0];

%Initial Optimized Input Values over the trajectory
optimal_input = zeros(1,params.N);

%Fmincon Options
options = optimoptions(@fmincon, 'TolFun', 0.001, 'MaxIter', 500, 'MaxFunEvals', 10000,...
                       'DiffMinChange', 0.001,'Display', 'iter', 'Algorithm', 'sqp');
                   
%Input Bounds
LB = -40*ones(1,params.N);
UB = 40*ones(1,params.N);

%Store Data
xHist = zeros(length(x),params.N);
%Solving
theta = theta0;
for ct = 1:params.N
    sprintf('iteration #: %d', ct)
    xHist(:,ct) = x;
    COSTFUN = @(u) acrobotObjectiveFCN(u, x, xf, optimal_input(:,1), params);
    CONSFUN = @(u) acrobotConstraintFCN(u, theta, xf, params);
    optimal_input = fmincon(COSTFUN, optimal_input, [], [], [], [], LB, UB, CONSFUN, options);

    [x, ~] = acrobotDynamicsDT(x, optimal_input(:,1), params); 
end
save('results', 'xHist', 'optimal_input');
%%
t = linspace(0, params.Ts*params.N, params.N);
figure();
subplot(2,2,1);
hold on;
plot(t, xf(1)*ones(1, params.N), 'k--');
plot(t,xHist(1,:),'b');
hold off;

subplot(2,2,2);
hold on;
plot(t, xf(2)*ones(1, params.N), 'k--');
plot(t,xHist(2,:),'b');
hold off;

subplot(2,2,3);
hold on;
plot(t, xf(3)*ones(1, params.N), 'k--');
plot(t,xHist(3,:),'b');
hold off;

subplot(2,2,4);
hold on;
plot(t, xf(4)*ones(1, params.N), 'k--');
plot(t,xHist(4,:),'b');
hold off;

figure();
plot(t, optimal_input);