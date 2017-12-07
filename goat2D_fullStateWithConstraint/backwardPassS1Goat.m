function dS1 = backwardPassS1Goat(t,S1,S2,x0,xd,ud,u0,Q,R,l)
    global T;
    global dt;
    tspan = 0:dt:(T-1)*dt;
    [~,dim] = size(x0);
    x0t = zeros(dim,1);
    xdt = zeros(dim,1);
    for i=1:1:dim
        x0t(i) = interp1(tspan,x0(:,i),t);
    end
    for i=1:1:dim
        xdt(i) = interp1(tspan,xd(:,i),t);
    end
    S2t = reshape(interp1(tspan,S2,t),12,12);
    udt = [interp1(tspan,ud(:,1),t);interp1(tspan,ud(:,2),t)];
    u0t = [interp1(tspan,u0(:,1),t);interp1(tspan,u0(:,2),t)];
    [A,B] = goatLinearizedFullStateDynamics(x0t,u0t,l);
    s1 = reshape(S1,12,1);
    dS1 = -(-2*Q*(xdt-x0t) + (A'-S2t*B*inv(R)*B')*s1 + 2*S2t*B*(udt-u0t));
    dS1 = reshape(dS1,12,1);
end