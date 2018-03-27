clear;
clc;
close all;
% addpath('lib')
% addpath('functions')
% profile on

% Define model parameters
params.m1 = 1;
params.m2 = 1;
params.l1 = 0.5;
params.l2 = 0.5; 
params.g = -9.81;
params.I1 = 0;
params.I2 = 0;
%Sample Time
params.Ts = 0.005;

% profile viewer
%Prediction Horizon
params.Total_steps = 50;
params.Duration = 2*params.Total_steps*params.Ts;
params.horizon = 5;

%Initial State
x0 = [pi+0.1; 0; 0; 0];
x = x0;
%Final State
xf = [pi; 0; 0; 0];

% State size
N_s = size(x0,1);
N_a = 1;

%Reference Trajectory
xref = zeros(N_s, params.Total_steps);

%p = zeros(5,params.N);

for i=1:4
    xref(i,:) = linspace(x0(i),xf(i),params.Total_steps);
end
%Fmincon Options
params.options = optimoptions(@fmincon, 'TolFun', 0.001, 'MaxIter', 500, 'MaxFunEvals', 10000,...
                       'DiffMinChange', 0.001,'Display', 'iter', 'Algorithm', 'sqp');
                   
%Input Bounds
params.LB = -20;
params.UB = 20;

% The final results will be stored here
xHist = zeros(N_s,params.Total_steps);
optimal_inputs = zeros(N_a, params.Total_steps);

% Set the initial states
u_prev = zeros(N_a, 1);
x_prev = x0;

%%
%Solving
for ct = 1:params.Total_steps
    sprintf('iteration #: %d', ct)
    %u_opt = zeros(N_a, params.horizon);
    %COSTFUN = @(p) acrobotObjectiveFCN(p, xf, uopt(:,1), params);

		%set up the referece trajectory
		cend = ct + params.horizon - 1;
		if cend > params.Total_steps 
			cend = params.Total_steps;
		end
		xref_k = xref(:, ct:cend);
		% number of parameters in this loop
		n = size(xref_k,2);
		u_opt = zeros(N_a, n); 
		LB = ones(N_a, n) * params.LB;
		UB = ones(N_a, n) * params.UB;

		COSTFUN = @(u_opt) ObjectiveFCN_test(u_opt, u_prev, x_prev, xref_k, params, @acrobotDynamicsCT);
    CONSFUN = @(u_opt) acrobotConstraintFCN_DC(u_opt, x0, xf, params);
    u_opt = fmincon(COSTFUN, u_opt, [], [], [], [], LB, UB, CONSFUN, params.options);
    %u_opt = fmincon(COSTFUN, u_opt, [], [], [], [], params.LB, params.UB, CONSFUN, params.options);
   	
		% Use the first entry of u_opt as the read input to the system
		u_k = u_opt(:,1);
		% Update the state based on optimal control input
		x_k = ode45(@(t,x)acrobotDynamicsCT(t,x,u_prev,params),[0,params.Ts],x_prev);
		x_k = x_k.y(:,end);	

		
    xHist(:,ct) = x_k;
    optimal_inputs(:,ct) = u_k;
		
		u_prev = u_k;	
		x_prev = x_k;
	

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
t = linspace(0, params.Duration, params.Total_steps);
figure();
subplot(2,2,1);
title('$\theta_1$','interpreter','latex')
hold on;
plot(t, xref(1,:), 'k--');
plot(t,xHist(1,:),'b');
hold off;

subplot(2,2,2);
hold on;
plot(t, xref(2,:), 'k--');
plot(t,xHist(2,:),'b');
title('$\dot{\theta_1}$', 'interpreter','latex')
hold off;

subplot(2,2,3);
hold on;
plot(t, xref(3,:), 'k--');
plot(t,xHist(3,:),'b');
title('$\theta_2$','interpreter','latex')
hold off;

subplot(2,2,4);
hold on;
plot(t, xref(4,:), 'k--');
plot(t,xHist(4,:),'b');
title('$\dot{\theta_2}$', 'interpreter','latex')
hold off;

figure(2);
plot(t, optimal_inputs);
title('optimal inputs');
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
