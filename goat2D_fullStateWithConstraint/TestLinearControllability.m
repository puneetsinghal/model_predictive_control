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

testtheta = [1.0781; 1.0701; 0.993] + 0.005;
testthetaRest = findFeasibleConfigurationAnalytical(testtheta,l);
testtheta = [testtheta; testthetaRest];
testthetadot = [0;0;0;0;0;0];
qtest = [testtheta;testthetadot];

u = zeros(2,1);
uAna = zeros(6,1); uAnatest = zeros(6,1); uAnatest(3) = 0.01; uAnatest(6) = 0.01;
utest = zeros(2,1)+0.01;

%% Check the accuracy of linearization

qdotAnatest = goatFullDynamicsWithConstraints(qtest,uAnatest,l);
qdotAna = goatFullDynamicsWithConstraints(qstart,uAna,l);

[A,B] = goatLinearizedFullStateDynamics(qstart,u,l);
qdotLin = qdotAna + A*(qtest - qstart) + B*(u - utest);

difference = norm(qdotAnatest - qdotLin);
fprintf('Difference in %f\n',difference);


