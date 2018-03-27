
params.m1 = 1;
params.m2 = 1;
params.l1 = 0.5;
params.l2 = 0.5; 
params.g = -9.81;
params.I1 = 0;
params.I2 = 0;

x0 = [1;0;0;0];
params.Ts = 0.001;
u = 0;  


results = ode45(@(t,x)acrobotDynamicsCT(t, x, u, params), linspace(0,4,4/params.Ts), x0);

N = size(results.x,2);
energy = zeros(N,1);

for i=1:N
    x = results.y(:,i);
    drawAcrobot(1, x, params);
    KE = 0.5*params.m1*params.l1^2/4*x(3)^2 + ...
        params.m2*(0.5*params.l1^2*x(3)^2 + 0.5*params.l2^2/4*(x(3) + x(4))^2 + params.l1*params.l2/2*x(3)*(x(3) + x(4))*cos(x(2)))+...
        0.5*params.I1*x(3)^2 + 0.5*params.I2*x(4)^2;
    
    PE = params.m1*params.l1/2*params.g*(cos(x(1))) + params.m2*params.g*(params.l1*cos(x(1))+ params.l2/2*cos(x(1) + x(2)));
    energy(i) = KE + PE;
end
figure();
plot(energy)