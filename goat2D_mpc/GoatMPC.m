clear;
clc;
profile on

addpath('lib')

%Sample Time
Ts = 0.001;
params.Ts = Ts;
%Prediction Horizon
N = 20;
params.N = N;
link_length = [1.0 0.5 0.2];

%Initial State
q20 = [1.0781, 1.0701, 0.9933]';
dq20 = [0, 0, 0]';
q10 = [2.0634, -1.0701, -0.9933]';
% x = [q20; dq20; q10] + [0.012; 0; 0; 0; 0; 0; 0 ; 0; 0];
x = [q20; dq20] + [0.001; 0; 0; 0; 0; 0];

%Final State
%xf = [pi;0];
xf = [q20; dq20];

%Initial Optimized Input Values over the trajectory
optimal_input = zeros(2,N);

%Fmincon Options
options = optimoptions(@fmincon,'TolFun',0.001,'MaxIter',50,'MaxFunEvals',10000,...
                       'DiffMinChange',0.001,'Display','iter','Algorithm','interior-point');
                   
%Input Bounds
LB = -10*ones(2,N);
UB = 10*ones(2,N);

%Store Data
xHist = zeros(length(x),N);
%Solving
for ct = 1:N
    sprintf('iteration #: %d', ct)
    xHist(:,ct) = x;
    COSTFUN = @(u) cost(u, x, xf, N, optimal_input(:,1), Ts, link_length);
    CONSTFUN = @(u) goatConstraintFCN(u,x,xf,Ts,N, link_length);
    optimal_input = fmincon(COSTFUN,optimal_input,[],[],[],[],LB,UB,CONSTFUN,options);

    [x, y] = goatDynamicsDT(x, optimal_input(:,1), Ts, link_length); 
%     x = [(x(1:6) + dx*ts); x10];
end
save('results', 'xHist', 'optimal_input');
%%
t = linspace(0,params.Ts*params.N, params.N);
figure();
subplot(2,3,1);
hold on;
plot(t, xf(1)*ones(1,params.N), 'k--');
plot(t,xHist(1,:),'b');
hold off;

subplot(2,3,2);
hold on;
plot(t, xf(2)*ones(1,params.N), 'k--');
plot(t,xHist(2,:),'b');
hold off;

subplot(2,3,3);
hold on;
plot(t, xf(3)*ones(1,params.N), 'k--');
plot(t,xHist(3,:),'b');
hold off;


subplot(2,3,4);
hold on;
plot(t, xf(4)*ones(1,params.N), 'k--');
plot(t,xHist(4,:),'b');
hold off;

subplot(2,3,5);
hold on;
plot(t, xf(5)*ones(1,params.N), 'k--');
plot(t,xHist(5,:),'b');
hold off;

subplot(2,3,6);
hold on;
plot(t, xf(4)*ones(1,params.N), 'k--');
plot(t,xHist(6,:),'b');
hold off;

figure();
plot(t, optimal_input);

%% Cost Function
function J = cost(u, xk, xref, N, u0, Ts, link_length)

    Q = [10 0 0 0 0 0;...
            0 10 0 0 0 0;...
            0 0 10 0 0 0;...
            0 0 0 1 0 0;...
            0 0 0 0 1 0;...
            0 0 0 0 0 1];
    R = 0.01*eye(2);

    J = 0;
    xk1 = xk;


    for i = 1:N
        uk = u(:,i);
        xk1 = goatDynamicsDT(xk1, uk, Ts, link_length);
%         xkd = xk1(1:6);
        
% %         J = J + (xkd-xref)'*Q*(xkd-xref);
        J = J + (xk1-xref)'*Q*(xk1-xref);
        if i ==1
            J = J + (uk-u0)' * R * (uk-u0);
        else
            J = J + (uk-u(i-1))' * R * (uk-u(i-1));
        end
    end  
%     if(norm(xkd-xf)>0.001)
%         J = NaN;
%     end
    
end