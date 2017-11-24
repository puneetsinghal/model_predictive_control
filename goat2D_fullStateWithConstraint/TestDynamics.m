clear;
close all;
clc;

%% Initialize

l = [1,0.5,0.2];
starttheta = [1.0781; 1.0701; 0.993];
startthetaRest = findFeasibleConfigurationAnalytical(starttheta,l);
starttheta = [starttheta; startthetaRest];
startthetadot = zeros(6,1);
qstart = [starttheta;startthetadot];
T = 5;
dt = 0.1;
u = zeros(6,1);

%% Integrate the trajectory

opt = odeset('AbsTol',1e-8,'RelTol',1e-8);
tspan = 0:dt:T;
[t,q] = ode45(@(t,q) goatFDWC(t,q,u,l),tspan,qstart,opt);

%% Visualize Trajectory

visualizeTrajectory(q,l);

