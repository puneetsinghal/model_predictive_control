clear;
clc;
close all;
profile on
addpath('lib')

%Sample Time
Ts = 0.001;
params.Ts = Ts;

%Prediction Horizon
N = 10;
params.N = N;
link_length = [1.0 0.5 0.2];

%Initial State
q20 = [1.0781, 1.0701, 0.9933]';
dq20 = [0, 0, 0]';
q10 = [2.0634, -1.0701, -0.9933]';
%x = [q20; dq20] + [0.12; 0.56; -0.58; 0; 0; 0];
x = [q20; dq20] + [0.012; 0; 0; 0; 0; 0];

%Final State
xf = [q20; dq20];

%Initial Optimized Input Values over the trajectory
optimal_parameters = zeros(8,N);
p0 = zeros(8,N);
p0(1:6,:) = x + (xf-x)*(0:1:(N-1))/(N-1);

%Fmincon Options
options = optimoptions(@fmincon,'StepTolerance',1e-15,'TolFun',1e-8,'MaxIter',10000,'MaxFunEvals',500000,...
                       'DiffMinChange',1e-3,'Display','iter','Algorithm','sqp');
                   
%Input Bounds
lb_input = -100;
ub_input = 100;
% LB_ = [pi/5  ;pi/5  ;pi/6  ;-inf;-inf;-inf;lb_input;lb_input];
% UB_ = [4*pi/5;4*pi/5;5*pi/6;inf ;inf ;inf ;ub_input;ub_input];
LB_ = [0    ;0  ;0  ;-inf;-inf;-inf;lb_input;lb_input];
UB_ = [pi/2 ;pi ;pi/2 ;inf ;inf ;inf ;ub_input;ub_input];

LB = LB_;
UB = UB_;

for i = 4:3:3*N
    LB = [LB ,LB_];
    UB = [UB ,UB_];
end

%Store Data
xHist = zeros(length(x),N);

%Solving
% for ct = 1:N
%     sprintf('iteration #: %d', ct)
%     xHist(:,ct) = x;
    COSTFUN = @(p) cost(p, x, xf, N, optimal_parameters(7:8,1), Ts, link_length);
    CONSTFUN = @(p) goatConstraintFCNDC(p,x,xf,Ts,N, link_length);
    optimal_parameters = fmincon(COSTFUN,p0,[],[],[],[],LB,UB,CONSTFUN,options);

%     [x, y] = goatDynamicsDT(x, optimal_parameters(7:8,1), Ts, link_length); 
% %     x = [(x(1:6) + dx*ts); x10];
% end
%save('results', 'xHist', 'optimal_input');
%%
t = linspace(0,params.Ts*params.N, params.N);
figure();
subplot(2,3,1);
hold on;
plot(t, xf(1)*ones(1,params.N), 'k--');
plot(t,optimal_parameters(1,:),'b');
hold off;

subplot(2,3,2);
hold on;
plot(t, xf(2)*ones(1,params.N), 'k--');
plot(t,optimal_parameters(2,:),'b');
hold off;

subplot(2,3,3);
hold on;
plot(t, xf(3)*ones(1,params.N), 'k--');
plot(t,optimal_parameters(3,:),'b');
hold off;

subplot(2,3,4);
hold on;
plot(t, xf(4)*ones(1,params.N), 'k--');
plot(t,optimal_parameters(4,:),'b');
hold off;

subplot(2,3,5);
hold on;
plot(t, xf(5)*ones(1,params.N), 'k--');
plot(t,optimal_parameters(5,:),'b');
hold off;

subplot(2,3,6);
hold on;
plot(t, xf(4)*ones(1,params.N), 'k--');
plot(t,optimal_parameters(6,:),'b');
hold off;

figure();
hold on
plot(t, optimal_parameters(7,:));
plot(t, optimal_parameters(8,:));

%% Cost Function
function J = cost(p, xk, xref, N, u0, Ts, link_length)

    Q = [100 0 0 0 0 0;...
       	 0 100 0 0 0 0;...
         0 0 100 0 0 0;...
         0 0 0 1 0 0;...
         0 0 0 0 1 0;...
         0 0 0 0 0 1];
    R = 0.01*eye(2);

    J = 0;
    xk1 = xk;


    for i = 1:N
        uk = p(7:8,i);
        xk1 = goatDynamicsDT(xk1, uk, Ts, link_length);

%      J = J + (xk1-xref)'*Q*(xk1-xref);
%      J = J + (uk)' * R * (uk);
%     end

        
        
        J = J + (xk1-xref)'*Q*(xk1-xref);
        if i ==1
            J = J + (uk-u0)' * R * (uk-u0);
        else
            J = J + (uk-p(7:8,i-1))' * R * (uk-p(7:8,i-1));
        end
    end
    
end