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
params.N = 10;


%Initial State
theta0 = [0.6; 0.1; 0; 0];
x = theta0;
%Final State
xf = [0; 0; 0; 0];
numPoints = 10;
trajDesired = zeros(4,numPoints);

for i=1:4
    trajDesired(i,:) = linspace(theta0(i), xf(i), numPoints);
end

time = linspace(0, numPoints*params.Ts*20, numPoints + 1);
%Initial Optimized Input Values over the trajectory
optimal_input = zeros(1,params.N);

%Fmincon Options
options = optimoptions(@fmincon, 'TolFun', 0.001, 'MaxIter', 500, 'MaxFunEvals', 10000,...
                       'DiffMinChange', 0.001,'Display', 'iter', 'Algorithm', 'sqp');
                   
%Input Bounds
LB = -10*ones(1,params.N);
UB = 10*ones(1,params.N);

%Store Data
xHist = zeros(length(x),params.N);
%Solving
theta = theta0;
for ct = 1:params.N
    sprintf('iteration #: %d',ct)
    xHist(:,ct) = x;
    COSTFUN = @(u) acrobotCost(u, x, xf, optimal_input(:,1), params);
    CONSTFUN = @(u) acrobotConstraintFCN_traj(u, theta, trajDesired, time, params);
    optimal_input = fmincon(COSTFUN, optimal_input, [], [], [], [], LB, UB, CONSTFUN, options);

    [x, ~] = acrobotDynamicsDT(x, optimal_input(:,1), params); 
end
save('results', 'xHist', 'optimal_input');
%%
t = linspace(0,params.Ts*params.N, params.N);
figure();
subplot(2,2,1);
hold on;
plot(t, xf(1)*ones(1,params.N), 'k--');
plot(t,xHist(1,:),'b');
hold off;

subplot(2,2,2);
hold on;
plot(t, xf(2)*ones(1,params.N), 'k--');
plot(t,xHist(2,:),'b');
hold off;

subplot(2,2,3);
hold on;
plot(t, xf(3)*ones(1,params.N), 'k--');
plot(t,xHist(3,:),'b');
hold off;

subplot(2,2,4);
hold on;
plot(t, xf(4)*ones(1,params.N), 'k--');
plot(t,xHist(4,:),'b');
hold off;

figure();
plot(t, optimal_input);
%%
%Cost Function
function J = acrobotCost(u, xk, xref, u0, params)

    Q = diag([10000; 10000; 0.01; 0.01]); %[10000*eye(4), zeros(4);zeros(4), 0.01*eye(4)];
    R = 0.01*eye(1);

    J = 0;
    xk1 = xk;
    N = params.N;

    for i = 1:N
        uk = u(:,i);
        xk1 = acrobotDynamicsDT(xk1, uk, params);
        
%         J = J + (xk1-xref)'*Q*(xk1-xref);
        J = J + (xk1)'*Q*(xk1);

        if i ==1
            J = J + (uk-u0)' * R * (uk-u0);
        else
            J = J + (uk-u(i-1))' * R * (uk-u(i-1));
        end
    end  
%     if (norm(xk1(1:4) - xref(1:4)) > 0.001)
%         J = 10000;
%     end    
end