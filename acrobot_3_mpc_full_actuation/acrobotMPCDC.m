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
params.g = 0;
params.I1 = 0;
params.I2 = 0;
%Sample Time
params.Ts = 0.01;

% profile viewer
%Prediction Horizon
params.N = 60;
params.Duration = 1*params.N*params.Ts;
params.M = floor(params.Duration/params.Ts);
%Initial State
x0 = [pi/2-0.1; 0; 0; 0; 0; 0];
%Final State
xf = [pi/2; 0; 0; 0; 0; 0];
p = zeros(9,params.M);
p(1:6,:) = repmat(x0,1,params.M);

% for i=1:6
%     p(i,1:params.M) = linspace(x0(i),xf(i),params.M);
%     p(i,params.M+1:end) = xf(i);
% end
%Fmincon Options
options = optimoptions(@fmincon, 'TolFun', 0.0001, 'MaxIter', 5000, 'MaxFunEvals', 10000,...
                       'DiffMinChange', 0.0001,'Display', 'iter', 'Algorithm', 'sqp');
                   
%Input Bounds
inputBounds = 10;
% LB = [-inf*ones(4,params.N); -20*ones(1,params.N)];
% UB = [inf*ones(4,params.N); 20*ones(1,params.N)];
LB = [-2*pi*ones(3,params.N); -inf*ones(3,params.N); -inputBounds*ones(3,params.N)];
UB = [ 2*pi*ones(3,params.N); inf*ones(3,params.N); inputBounds*ones(3,params.N)];

%%
%Solving
% for ct = 1:params.M
%     sprintf('iteration #: %d', ct)
    uopt = zeros(1,params.N);
%     p1 = p(:,ct:ct+params.N-1);
    COSTFUN = @(x) acrobotObjectiveFCN(x, xf, uopt(:,1), params);
    CONSFUN = @(x) acrobotConstraintFCN_DC(x, x0, xf, params);
    p = fmincon(COSTFUN, p, [], [], [], [], LB, UB, CONSFUN, options); 
%     p(:,ct:ct+params.N-1) = p1;
% end
xHist = p(1:6,:);
optimal_inputs = p(7,:);

save('results', 'xHist', 'optimal_inputs');
%%
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
%     pause(0.1);
end