% clear;
% clc;
% close all;
% addpath('lib')
% addpath('functions')
% profile on

params.m1 = 1;
params.m2 = 1;
params.l1 = 0.5;
params.l2 = 0.5; 
params.g = -9.81;
params.I1 = 0;
params.I2 = 0;
%Sample Time
params.Ts = 0.1;

% profile viewer
%Prediction Horizon
params.N = 10;
params.Duration = 1;
params.M = floor(params.Duration/params.Ts) + params.N;
%Initial State
x0 = [pi-0.3; 0; 0; 0];
x = x0;
%Final State
xf = [pi; 0; 0; 0];
% p = zeros(5,params.N);

% for i=1:4
%     p(i,:) = linspace(x0(i),xf(i),params.N);
% end
%Fmincon Options
params.options = optimoptions(@fmincon, 'TolFun', 0.001, 'MaxIter', 200, 'MaxFunEvals', 1000,...
                       'DiffMinChange', 0.001,'Display', 'iter', 'Algorithm', 'sqp');
                   
%Input Bounds
params.LB = -20*ones(1,params.N);
params.UB = 20*ones(1,params.N);

xHist = zeros(4,params.M);
optimal_inputs = zeros(1, params.M);
uopt = zeros(1, params.N);
%%
%Solving
u0 = 0;
xHist(:,1) = x0;
optimal_inputs(:,1) = u0;
for ct = 1:params.M - params.N +1
    sprintf('iteration #: %d', ct)
    
    COSTFUN = @(u) acrobotObjectiveFCN(@acrobotDynamicsCT, u, x0, xf, u0, params);
    CONSFUN = @(u) acrobotConstraintFCN_DC(@acrobotDynamicsCT, u, x0, xf, params);
    uopt = fmincon(COSTFUN, optimal_inputs, [], [], [], [], params.LB, params.UB, CONSFUN, params.options);
    u0 = uopt(:,1);
%     for i =1:params.N
    results = ode45(@(t,x)acrobotDynamicsCT(t, x, u0, params),linspace(0, params.Ts), x0);
    xHist(:,ct+1) = results.y(:,end);
    optimal_inputs(:,ct+1) = u0;
%     end
%     xHist(:,ct:ct+params.N-1) = p(1:4,:);
%     optimal_inputs(:,ct:ct+params.N-1) = p(5,:);

%     [x, ~] = acrobotDynamicsDT(x, uopt(:,1), params);
%     optimal_inputs(:,ct) = uopt(:,1);
end
% xHist = p(1:4,:);
% optimal_inputs = p(5,:);

save('results', 'xHist', 'optimal_inputs', 'params', 'x0', 'xf');
%%
% load('results/swingup_nice.mat');
% load('results/working_swingup.mat');
% load('results/mpdcdc_nice.mat');
pause(1);
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

% Animate the results:
figure();
for i = 1:length(t)
    drawAcrobot(t(i), xHist(:,i), params);
%     if (i==1)
%         pause(5);
%     end
    pause(0.01);
end