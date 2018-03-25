
params.m1 = 1;
params.m2 = 1;
params.l1 = 0.5;
params.l2 = 0.5; 
params.g = -9.81;
params.I1 = 0;
params.I2 = 0;

x = [0.1;0;0;0];
ts = 0.01;
u = 0;


for i = 1:1000
    dx = acrobotDynamicsCT(x, u, params);
    x = x + dx*ts;
    drawAcrobot(1, x, params);
%     pause(0.01);
end