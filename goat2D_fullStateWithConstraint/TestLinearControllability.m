clear;
close all;
clc;

%% Initialize

l = [1.0 0.5 0.2];
starttheta = [1.0781; 1.0701; 0.993];
startthetaRest = findFeasibleConfigurationAnalytical(starttheta,l);
starttheta = [starttheta; startthetaRest];
startthetadot = [0;0;0;0;0;0];
qstart = [starttheta;startthetadot];

u = zeros(2,1);
uAna = zeros(6,1);

%% Check the accuracy of linearization

qdotAna = goatFullDynamicsWithConstraints(qstart,uAna,l);
[A,B] = goatLinearizedFullStateDynamics(qstart,u,l);
qdotLin = A*qstart + B*u;

difference = norm(qdotAna - qdotLin);
fprintf('Difference in %f\n',difference);


