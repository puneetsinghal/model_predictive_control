clear;
clc;
addpath('lib')
addpath('functions')

%Sample Time
Ts = 0.001;

%Prediction Horizon
N = 10;

parameters();

%Initial State
theta0 = [0; 0; 0; 0; 0; 0; 0; 0];
x = theta0;
%Final State
thetaf = [0.01; 0.01; 0.01; 0.01; 0; 0; 0; 0];

%Initial Optimized Input Values over the trajectory
optimal_input = zeros(4,N);

%Fmincon Options
options = optimoptions(@fmincon, 'TolFun', 0.001, 'MaxIter', 50, 'MaxFunEvals', 10000,...
                       'DiffMinChange', 0.001,'Display', 'iter', 'Algorithm', 'sqp');
                   
%Input Bounds
LB = -4*ones(4,N);
UB = 4*ones(4,N);

%Store Data
xHist = zeros(length(x),N);
%Solving
theta = theta0;
for ct = 1:N
    ct
    xHist(:,ct) = x;
    costfun = @(u) cost(u, x, thetaf, N, optimal_input(:,1), Ts, m, COM, I, y, z, L1);
    constrfun = @(u) constraintFCN(u, theta, thetaf, Ts, N, m, COM, I, y, z, L1);
    optimal_input = fmincon(costfun, optimal_input, [], [], [], [], LB, UB, constrfun, options);

    [x, ~] = dynamicsDT(m, COM, I, y, z, L1, x, optimal_input(:,1), Ts); 
%     x = [(x(1:6) + dx*ts); x10];
    save('results', 'xHist', 'optimal_input');
end

%Cost Function
function J = cost(u, xk, xref, N, u0, Ts, m, COM, I, y, z, L1)

    Q = [10000*eye(4), zeros(4);zeros(4), 0.01*eye(4)];
    R = 0.01*eye(4);

    J = 0;
    xk1 = xk;


    for i = 1:N
        uk = u(:,i);
        xk1 = dynamicsDT(m, COM, I, y, z, L1, xk1, uk, Ts);
        
        J = J + (xk1-xref)'*Q*(xk1-xref);

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

%Constraint Function
function [c,ceq] = constraint(u, x, xref, Ts, N, m, COM, I, y, z)
c = [];
ceq = [];

end