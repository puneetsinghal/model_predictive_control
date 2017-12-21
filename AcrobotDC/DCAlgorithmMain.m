%Sample Time
ts = 0.1;

%Prediction Horizon
N = 50;

%Initial State
x0 = [0;0;0;0];

%Final State
xf = [pi;0;0;0];

%Initial Parameter Values over the trajectory
p0 = zeros(5,N);

%Fmincon Options
options = optimoptions(@fmincon,'TolFun',1e-6,'MaxIter',10000,'MaxFunEvals',100000,'Display','iter',...
                       'DiffMinChange',1e-3,'Algorithm','interior-point');
                   
%Bounds
lb_input = -100;
ub_input = 100;
LB = [-Inf;-Inf;-Inf;-Inf;lb_input];
UB = [Inf;Inf;Inf;Inf;ub_input];
for i = 1:N-1
    LB = [LB [-Inf;-Inf;-Inf;-Inf;lb_input]];
    UB = [UB [Inf;Inf;Inf;Inf;ub_input]];
end

%Store Data
xHist = x0;
uHist = 0;

%Solving
costfun = @(p) cost(p,N,xf,ts);
constrfun = @(p) constraint(x0,xf,p,ts,N);
optimal_parameters = fmincon(costfun,p0,[],[],[],[],LB,UB,constrfun,options);


%Extracting states
xHist = optimal_parameters(1:4,:);
uHist = optimal_parameters(5,:);

figure
title('Parameters Obtained from Direct Collocation on Discretized Dynamics');
subplot(2,2,1)
plot(1:N,xHist(1,:))
title('X1 State trajectory')
xlabel('Time(t)')
ylabel('$\theta_1$','interpreter','latex')
ylim([-pi 5*pi/4])
yticks([-pi:pi/4:2*pi])
yticklabels({'-\pi','-\3*pi/4','-\pi/2','-\pi/4','0','\pi/4','\pi/2','3*\pi/4','\pi','5*\pi/4'})
grid on

subplot(2,2,2)
plot(1:N,xHist(2,:))
title('X2 State trajectory')
xlabel('Time(t)')
ylabel('$\theta_2$','interpreter','latex')
ylim([-pi 5*pi/4])
yticks([-pi:pi/4:2*pi])
yticklabels({'-\pi','-3*\pi/4','-\pi/2','-\pi/4','0','\pi/4','\pi/2','3*\pi/4','\pi','5*\pi/4'})
grid on

subplot(2,2,3)
plot(1:N,xHist(3,:))
title('X3 State trajectory')
xlabel('Time(t)')
ylabel('$\dot{\theta}_1$','interpreter','latex')
grid on

subplot(2,2,4)
plot(1:N,xHist(4,:))
title('X4 State trajectory')
xlabel('Time(t)')
ylabel('$\dot{\theta}_2$','interpreter','latex')
grid on

figure
plot(1:N,uHist(1,:))
title('Control Input')
xlabel('Time(t)')
ylabel('$u$','interpreter','latex')
grid on

xk = x0;
xHist1 = xk;
uk = uHist(1,1);
for i = 1:N
    xint = DCdynamics(xk,uk,ts);
        if i<N
        uk = uHist(1,i+1);
    end
    xHist1 = [xHist1 xint];
    xk = xint;
end

figure
for i = 1:N+30
    
    if i>N
        xHist(1,i) = xHist(1,N);
        xHist(2,i) = xHist(2,N);
    end
    hold on
    cla
    plot([0 sin(xHist(1,i))] ,[0 -cos(xHist(1,i))],'bs-');
    plot([sin(xHist(1,i)) (sin(xHist(1,i))+sin(xHist(2,i)+xHist(1,i)))] ,[-cos(xHist(1,i)) (-cos(xHist(1,i))-cos(xHist(2,i)+xHist(1,i)))],'bo-');
    axis([-2 2 -2 2])
    title(['Simulation of Cart : Actual Time = ' num2str(i*ts)]);
    grid on
    pause(0.2)
end



%Cost Function
function J = cost(p,N,xf,ts)
J = 0;
Q = 10*eye(4);
R = 1;
uk = p(5,1);
uk1 = p(5,2);
xk = p(1:4,1);
xk1 = p(1:4,2);
for i = 1:N-1 
    %Discretized Cost
    J = J + (xk)'*Q*(xk) + uk'*R*uk;
    %J = J + (xk'*Q*xk+xk1'*Q*xk1+uk*R*uk+uk1*R*uk1);
    if i<N-2
        uk = p(5,i+1);
        uk1 = p(5,i+2);
        xk = p(1:4,i+1);
        xk1 = p(1:4,i+2);
    end
end    
end

%Constraint Function
function [c,ceq] = constraint(x0,xf,p,ts,N)
c = [];
%Constarian Initial Point
ceq = p(1:4,1) - x0;
for tk = 1:N-1   
    
    xk = p(1:4,tk);
    uk = p(5,tk);
    xdotk = acrobotDynamics(xk,uk);
    
    xk1 = p(1:4,tk+1);
    uk1 = p(5,tk+1);
    xdotk1 = acrobotDynamics(xk,uk);
    
    xkc = (xk+xk1)/2 + ts*(xdotk - xdotk1)/8;
    ukc = (uk+uk1)/2;
    xdotkc = acrobotDynamics(xkc,ukc);
    
    %Defect
    delk = (xk - xk1) + ts*(xdotk+4*xdotkc+xdotk1)/6;
    ceq = [ceq delk];
end    
    %Constrain Final point
    ceq = [ceq [p(1:4,N)-xf]];

end
    
%System Dynamics
function xk1 = DCdynamics(x0,u,ts)
M = 1000;
delta = ts/M;
xk1 = x0;
for ct=1:M
    xk1 = xk1 + delta*acrobotDynamics(xk1,u);
end
end
