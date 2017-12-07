function dx = forwardPassTrajStabGoat(t,x,x0,u0,ud,S1,S2,R,l)
global T;
global dt;
tspan = 0:dt:(T-1)*dt;
[~,dim] = size(x0);
x0t = zeros(dim,1);
for i=1:1:dim
    x0t(i) = interp1(tspan,x0(:,i),t);
end
u0t = [interp1(tspan,u0(:,1),t);interp1(tspan,u0(:,2),t)];
udt = [interp1(tspan,ud(:,1),t);interp1(tspan,ud(:,2),t)];
S1t = interp1(tspan,S1,t)';
S2t = reshape(interp1(tspan,S2,t),12,12);
[~,B] = goatLinearizedFullStateDynamics(x0t,u0t,l);
% B = [zeros(12,8) B(:,1) zeros(12,2) B(:,2)];
ustar = udt - inv(R)*B'*(S2t*(x-x0t) + 0.5*S1t);
u = zeros(6,1);
u(3) = ustar(1);
u(6) = ustar(2);
dx = goatFullDynamicsWithConstraints(x,u,l);
end