clear;
close all;
clc;

%% Initialize

l = [1 0.5 0.2];
starttheta = [0.6781; 1.0701+0.4; 0.993-0.4];
startthetaRest = findFeasibleConfigurationAnalytical(starttheta,l);
starttheta = [starttheta; startthetaRest];
startthetadot = [0;0;0;0;0;0];
qstart = [starttheta;startthetadot];

finaltheta = [1.0781; 1.0701; 0.993];
finalthetaRest = findFeasibleConfigurationAnalytical(finaltheta,l);
finaltheta = [finaltheta; finalthetaRest];
finalthetadot = [0;0;0;0;0;0];
qfinal = [finaltheta;finalthetadot];

%% Nominal Trajectory
load('Saves/Ts0.01.mat','-mat','xHist');
load('Saves/Ts0.01.mat','-mat','uHist')
load('Saves/Ts0.01.mat','-mat','Ts');

xn = xHist';
un = uHist';
global dt;
dt = Ts;
clear xHist uHist Ts;
% xn should be 12xN
global T;
[T,~] = size(xn);

%% Desired Trajectory
% Desired trajectory - Linear interpoltion of start to goal
ud = zeros(size(un));
xd = qstart + (qfinal-qstart)*(0:1:(T-1))/(T-1);
xd = xd';    

%% Trajectory tracking
Q = (1e+1)*eye(12);
%Q(1:6,1:6) = (1e+1)*Q(1:6,1:6);
R = (1e-0)*eye(2);
opts = odeset('RelTol',1e-8,'AbsTol',1e-8);
tspan = (T-1)*dt:-dt:0;
xerror = sum(sqrt(sum((xn(:,1:6)-xd(:,1:6)).^2,2)));
xdoterror = sum(sqrt(sum((xn(:,7:12)-xd(:,7:12)).^2,2)));
xerrorlimit = 0.1;
xdoterrorlimit = 0.5;
count = 1;

while ((xerror > xerrorlimit) | (xdoterror > xdoterrorlimit))
count
[~,S2] = ode45(@(t,S2) backwardPassS2Goat(t,S2,xn,un,xd,Q,R,l), tspan, reshape(Q,144,1),opts);
fprintf('S2 backward done\n');
[~,S1] = ode45(@(t,S1) backwardPassS1Goat(t,S1,flip(S2,1),xn,xd,ud,un,Q,R,l),tspan,-2*Q*(xd(end,:)-xn(end,:))',opts);
fprintf('S1 backward done\n');
[t,x] = ode45(@(t,x) forwardPassTrajStabGoat(t,x,xn,un,ud,flip(S1,1),flip(S2,1),R,l),flip(tspan,2),xd(1,:)',opts);
fprintf('forward done\n');
un = computeActionsGoat(x,xn,un,ud,flip(S1,1),flip(S2,1),R,l);
fprintf('actions comoputed\n');
xn = x;
count = count+1;
xerror = sum(sqrt(sum((xn(:,1:6)-xd(:,1:6)).^2,2)))
xdoterror = sum(sqrt(sum((xn(:,7:12)-xd(:,7:12)).^2,2)))

end