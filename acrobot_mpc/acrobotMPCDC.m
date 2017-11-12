clear;
clc;
close all;
% addpath('lib')
% addpath('functions')
% profile on

params.m1 = 1;
params.m2 = 1;
params.l1 = 0.5;
params.l2 = 0.5; 
params.g = 9.81;
params.I1 = 0;
params.I2 = 0;
%Sample Time
params.Ts = 0.01;

% profile viewer
%Prediction Horizon
params.N = 10;
params.Duration = params.N*params.Ts;
params.M = floor(params.Duration/params.Ts);
%Initial State
x0 = [1.0; 1.0; 0; 0];
x = x0;
%Final State
xf = [pi; 0; 0; 0];

p = zeros(5,params.N);

%Fmincon Options
options = optimoptions(@fmincon, 'TolFun', 0.001, 'MaxIter', 500, 'MaxFunEvals', 10000,...
                       'DiffMinChange', 0.001,'Display', 'iter', 'Algorithm', 'sqp');
                   
%Input Bounds
LB = [-2*pi*ones(2,params.N); -inf*ones(2,params.N); -20*ones(1,params.N)];
UB = [ 2*pi*ones(2,params.N); inf*ones(2,params.N); 20*ones(1,params.N)];


%%
%Solving
theta = x0;
% for ct = 1:params.M
%     sprintf('iteration #: %d', ct)
%     xHist(:,ct) = x;
uopt = zeros(1,params.N);
COSTFUN = @(p) acrobotObjectiveFCN(p, xf, uopt(:,1), params);
CONSFUN = @(p) acrobotConstraintFCN_DC(p, x0, xf, params);
p = fmincon(COSTFUN, p, [], [], [], [], LB, UB, CONSFUN, options);

%     [x, ~] = acrobotDynamicsDT(x, uopt(:,1), params);
%     optimal_inputs(:,ct) = uopt(:,1);
% end
xHist = p(1:4,:);
optimal_inputs = p(5,:);

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
A.plotFunc = @(t,z)( drawAcrobot(t,z,params) );
A.speed = 0.25;
A.figNum = 101;
z = xHist;
z(2,:) = z(1,:) + xHist(2,:);
animate(t,z,A)
