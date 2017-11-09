clear;
clc;

addpath('lib')

%Sample Time
Ts = 0.001;

%Prediction Horizon
N = 10;

global global_link_length
global_link_length = [1.0 0.5 0.2];

%Initial State
q20 = [1.0781, 1.0701, 0.9933]';
dq20 = [0, 0, 0]';
q10 = [2.0634, -1.0701, -0.9933]';
x = [q20; dq20; q10] + [+0.12; 0.35; -0.76; 0.43; -0.8; 0.65; 0 ; 0; 0];
%x = 2*[1;1;1;1;1;1;0;0;0];
%Final State
xf = [q20; dq20];

%Initial Optimized Input Values over the trajectory
optimal_param = zeros(11,N);

%Fmincon Options
options = optimoptions(@fmincon,'TolFun',0.001,'MaxIter',100,'MaxFunEvals',1000,...
                       'DiffMinChange',0.001,'Display','iter','Algorithm','sqp');
                   
%Parameter Bounds
lb_input = -10;
ub_input = 10;
LB = [-Inf;-Inf;-Inf;-Inf;-Inf;-Inf;-Inf;-Inf;-Inf;lb_input;lb_input];
UB = [Inf;Inf;Inf;Inf;Inf;Inf;Inf;Inf;Inf;ub_input;ub_input];
for i = 2:1:N
    LB = [LB [-Inf;-Inf;-Inf;-Inf;-Inf;-Inf;-Inf;-Inf;-Inf;lb_input;lb_input]];
    UB = [UB [Inf;Inf;Inf;Inf;Inf;Inf;Inf;Inf;Inf;ub_input;ub_input]];
end
%Store Data
xHist = zeros(length(x),N);
%Solving
for ct = 1:N
    ct
    xHist(:,ct) = x;
    costfun = @(p) cost(p, xf, N, optimal_param(10:11,1), Ts);
    constrfun = @(p) constraint(p,xf,Ts,N);
    optimal_param = fmincon(costfun,optimal_param,[],[],[],[],LB,UB,constrfun,options);

    [x, y] = goatDynamicsDT(x, optimal_param(10:11,1), Ts); 
%     x = [(x(1:6) + dx*ts); x10];
end


% %Plot
% x0 = x;
% xHist = x;
% for i = 1:N
%     uk = optimal_input(:,i);
%     xint = goatDynamicsDT(x0,uk,Ts);
%     xHist = [xHist xint];
%     x0 = xint;
% end



%Cost Function
function J = cost(p, xref, N, u0, Ts)

    Q = [10 0 0 0 0 0;...
            0 10 0 0 0 0;...
            0 0 10 0 0 0;...
            0 0 0 1 0 0;...
            0 0 0 0 1 0;...
            0 0 0 0 0 1];
        Q = 10*eye(6);
    R = 0.01*eye(2);

    J = 0;
    xk1 = p(1:9,1);


    for i = 1:N
        uk = p(10:11,i);
        xk1 = goatDynamicsDT(xk1, uk, Ts)
        xkd = xk1(1:6);
        
        J = J + (xkd-xref)'*Q*(xkd-xref);

        if i ==1
            J = J + (uk-u0)' * R * (uk-u0);
        else
            J = J + (uk-p(10:11,i-1))' * R * (uk-p(10:11,i-1));
        end
    end  
%     if(norm(xkd-xf)>0.001)
%         J = NaN;
%     end
    
end

%Constraint Function
function [c,ceq] = constraint(p, xf, Ts, N)
c = [];
% ceq = [];

for tk = 0:1:N-2  
    
    xk = p(1:9,tk+1);
    uk = p(10:11,tk+1);
    xdotk = goatDynamicsCT(xk,uk);
    
    xk1 = p(1:9,tk+2);
    uk1 = p(10:11,tk+2);
    xdotk1 = goatDynamicsCT(xk1,uk1);
    
    xkc = (xk+xk1)/2 + Ts*(xdotk - xdotk1)/8;
    ukc = (uk+uk1)/2;
    xdotkc = goatDynamicsCT(xkc,ukc);
    %Defect
    delk = (xk - xk1) + Ts*(xdotk+4*xdotkc+xdotk1)/6;
    ceq = delk(1:6);
end    
    %Constrain Final point
    ceq = [ceq p(1:6,N)-xf(1:6)];

end