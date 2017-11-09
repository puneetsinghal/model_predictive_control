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
params.Duration = 2;
params.M = floor(params.Duration/params.Ts);
%Initial State
theta0 = [1.0; 0; 0; 0];
x = theta0;
%Final State
xf = [pi/2; 0; 0; 0];

%Initial Optimized Input Values over the trajectory
uopt = zeros(1,params.N);
optimal_inputs = zeros(1, params.M);
%Fmincon Options
options = optimoptions(@fmincon, 'TolFun', 0.001, 'MaxIter', 500, 'MaxFunEvals', 10000,...
                       'DiffMinChange', 0.001,'Display', 'iter', 'Algorithm', 'sqp');
                   
%Input Bounds
LB = -40*ones(1,params.N);
UB = 40*ones(1,params.N);

%Store Data
xHist = zeros(length(x),params.M);
%Solving
theta = theta0;
for ct = 1:params.M
    sprintf('iteration #: %d', ct)
    xHist(:,ct) = x;
    COSTFUN = @(u) acrobotObjectiveFCN(u, x, xf, uopt(:,1), params);
    CONSFUN = @(u) acrobotConstraintFCN(u, theta, xf, params);
    uopt = fmincon(COSTFUN, uopt, [], [], [], [], LB, UB, CONSFUN, options);

    [x, ~] = acrobotDynamicsDT(x, uopt(:,1), params);
    optimal_inputs(:,ct) = uopt(:,1);
end
save('results', 'xHist', 'optimal_inputs');
%%
t = linspace(0, params.Duration, params.M);
figure();
subplot(2,2,1);
hold on;
plot(t, xf(1)*ones(1, params.M), 'k--');
plot(t,xHist(1,:),'b');
hold off;

subplot(2,2,2);
hold on;
plot(t, xf(2)*ones(1, params.M), 'k--');
plot(t,xHist(2,:),'b');
hold off;

subplot(2,2,3);
hold on;
plot(t, xf(3)*ones(1, params.M), 'k--');
plot(t,xHist(3,:),'b');
hold off;

subplot(2,2,4);
hold on;
plot(t, xf(4)*ones(1, params.M), 'k--');
plot(t,xHist(4,:),'b');
hold off;

figure();
plot(t, optimal_inputs);

%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Plot the solution                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%HINT:  type help animate to figure out how to use the keyboard to interact
%with the animation (slow motion, pause, jump forward / backward...)

% Animate the results:
A.plotFunc = @(t,z)( drawAcrobot(t,z,dyn) );
A.speed = 0.25;
A.figNum = 101;
animate(t,xHist,A)
