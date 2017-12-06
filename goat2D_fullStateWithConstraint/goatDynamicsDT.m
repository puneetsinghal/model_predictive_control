function xk1 = goatDynamicsDT(xk, uk, Ts, l)
M = 10;
tspan = 0:Ts/M:Ts;
options = odeset('AbsTol',1e-8,'RelTol',1e-8);
u = zeros(6,1);
u(3) = uk(1);
u(6) = uk(2);
[~,xk1] = ode45(@(t,x) goatFDWC(t,x,u,l),tspan,xk,options);
xk1 = xk1(end,:)';
end