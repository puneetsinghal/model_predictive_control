function ustar = computeActionsGoat(x,x0,u0,ud,S1,S2,R,l)
    global T;
    ustar = zeros(T,2);
    for t = 1:1:T
        xt = x(t,:)';
        x0t = x0(t,:)';
        udt = ud(t,:)';
        u0t = u0(t,:)';
        [~,B] = goatLinearizedFullStateDynamics(x0t,u0t,l);
        S1t = S1(t,:)';
        S2t = reshape(S2(t,:),12,12);
        us = udt - inv(R)*B'*(S2t*(xt-x0t) + 0.5*S1t);
        ustar(t,:) = us';
    end
end