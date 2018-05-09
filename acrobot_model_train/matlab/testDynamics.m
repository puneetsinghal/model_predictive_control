
% acrobot configuration
params.m1 = 1;
params.m2 = 1;
params.l1 = 0.5;
params.l2 = 0.5; 
params.g = -9.81;
params.I1 = 0;
params.I2 = 0;


% Initial conditions
x0 = [2.0;0.1;0.1;0.1];
% Sampling time
ts = 0.01;
% No input, free motion
u = 0;
x_prev = x0;
% Test for 1000 time steps
for i = 1:1000
		
    x_k = ode45(@(t,x)acrobotDynamicsCT(t, x, u, params),[0, ts],x_prev);
		x_k = x_k.y(:,end); 
    drawAcrobot(i*ts, x_k, params);
		x_prev = x_k;
		pause(0.01);
end



