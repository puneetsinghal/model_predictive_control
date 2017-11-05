clear;
clc;
profile on

addpath('lib')

%Sample Time
Ts = 0.001;

%Prediction Horizon
N = 10;

link_length = [1.0 0.5 0.2];

%Initial State
q20 = [1.0781, 1.0701, 0.9933]';
dq20 = [0, 0, 0]';
q10 = [2.0634, -1.0701, -0.9933]';
x = [q20; dq20; q10] + [0.012; 0.35; -0.76; 0.43; -0.8; 0.65; 0 ; 0; 0];

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
    costfun = @(u) cost(u, x, xf, N, optimal_input(:,1), Ts, link_length);
    constrfun = @(u) constraintFCN(u,x,xf,Ts,N, link_length);
    optimal_input = fmincon(costfun,optimal_input,[],[],[],[],LB,UB,constrfun,options);

    [x, y] = goatDynamicsDT(x, optimal_input(:,1), Ts, link_length); 
%     x = [(x(1:6) + dx*ts); x10];
end
profile viewer
%Cost Function
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
        xkd = xk1(1:6);
        
        J = J + (xkd-xref)'*Q*(xkd-xref);

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