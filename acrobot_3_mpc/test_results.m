clear;
clc;
close all;
addpath('lib');
params.m1 = 1;
params.m2 = 1;
params.m3 = 1;
params.l1 = 0.5;
params.l2 = 0.5;
params.l3 = 0.5;
params.g = -9.81;
params.I1 = 0;
params.I2 = 0;
%Sample Time
params.Ts = 0.005;

% profile viewer
%Prediction Horizon
params.N = 10;
params.Duration = 10*params.N*params.Ts;
params.M = floor(params.Duration/params.Ts) + params.N;
%Initial State
x0 = [0; 0; 0; 0; 0; 0];
x = x0;
%Final State
xf = [pi/2; 0; 0; 0; 0; 0];
p = zeros(7,params.M);

for i=1:6
    p(i,:) = linspace(x0(i),xf(i),params.M);
end
%Fmincon Options
options = optimoptions(@fmincon, 'TolFun', 0.0001, 'MaxIter', 500, 'MaxFunEvals', 10000,...
                       'DiffMinChange', 0.0001,'Display', 'iter', 'Algorithm', 'sqp');
                   
%Input Bounds
% LB = [-inf*ones(4,params.N); -20*ones(1,params.N)];
% UB = [inf*ones(4,params.N); 20*ones(1,params.N)];
LB = [-2*pi*ones(2,params.N); -inf*ones(4,params.N); -20*ones(1,params.N)];
UB = [ 2*pi*ones(2,params.N); inf*ones(4,params.N); 20*ones(1,params.N)];

%%
load results.mat
t = linspace(0, params.Duration, params.M);
figure();
subplot(2,3,1);
hold on;
plot(t, xf(1)*ones(1, params.M), 'k--');
plot(t,xHist(1,:),'b');
hold off;

subplot(2,3,2);
hold on;
plot(t, xf(2)*ones(1, params.M), 'k--');
plot(t,xHist(2,:),'b');
hold off;

subplot(2,3,3);
hold on;
plot(t, xf(3)*ones(1, params.M), 'k--');
plot(t,xHist(3,:),'b');
hold off;

subplot(2,3,4);
hold on;
plot(t, xf(4)*ones(1, params.M), 'k--');
plot(t,xHist(4,:),'b');
hold off;

subplot(2,3,5);
hold on;
plot(t, xf(5)*ones(1, params.M), 'k--');
plot(t,xHist(5,:),'b');
hold off;

subplot(2,3,6);
hold on;
plot(t, xf(6)*ones(1, params.M), 'k--');
plot(t,xHist(6,:),'b');
hold off;

figure();
plot(t, optimal_inputs);

%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Plot the solution                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%HINT:  type help animate to figure out how to use the keyboard to interact
%with the animation (slow motion, pause, jump forward / backward...)

% Animate the results:
% A.plotFunc = @(t,z)( drawAcrobot(t,z,params) );
% A.speed = 0.25;
% A.figNum = 101;
% z = xHist;
% z(2,:) = z(1,:) + xHist(2,:);
% animate(t,z,A)
for i = 1:length(t)
    drawAcrobot(t(i), xHist(:,i), params);
%     if (i==1)
%         pause(5);
%     end
    pause(0.01);
end