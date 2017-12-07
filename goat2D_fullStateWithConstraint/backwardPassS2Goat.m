function dS2 = backwardPassS2Goat(t,S2,x0,u0,xd,Q,R,l)
    global T;
    global dt;
    tspan = 0:dt:(T-1)*dt;
    [~,dim] = size(x0);
    x0t = zeros(dim,1);
    for i=1:1:dim
        x0t(i) = interp1(tspan,x0(:,i),t);
    end
    
    u0t = [interp1(tspan,u0(:,1),t);interp1(tspan,u0(:,2),t)];
    [A,B] = goatLinearizedFullStateDynamics(x0t,u0t,l);
    s2 = reshape(S2,12,12);
    dS2 = -(Q - s2*B*inv(R)*B'*s2 + s2*A + A'*s2);
    dS2 = reshape(dS2,144,1);
end