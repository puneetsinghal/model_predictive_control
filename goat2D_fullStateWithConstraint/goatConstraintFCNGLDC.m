function [c, ceq] = goatConstraintFCNGLDC(p, x, xref, Ts, N, l)
%% Nonlinear MPC design parameters
% Range of theta posn
%Constarian Initial Point
c = [];
ceq = p(1:12,1) - x;
for tk = 1:N-1
    xk = p(1:12,tk);
    u = p(13:14,tk);
    uk = zeros(6,1); uk(3,1) = u(1,1); uk(6,1) = u(2,1);
    xdotk = goatFullDynamicsWithConstraints(xk,uk,l);
    
    xk1 = p(1:12,tk+1);
    u = p(13:14,tk+1);
    uk1 = zeros(6,1); uk1(3,1) = u(1,1); uk1(6,1) = u(2,1);
    xdotk1 = goatFullDynamicsWithConstraints(xk1,uk1,l);
    del0 = findFeasibleConfigurationAnalytical(xk1(1:3,1),l) - xk1(4:6,1);
    
    xkc = (xk+xk1)/2 + Ts*(xdotk - xdotk1)/8;
    ukc = (uk+uk1)/2;
    xdotkc = goatFullDynamicsWithConstraints(xkc,ukc,l);
    
    x1 = ((39*sqrt(21)+231)*xk + 224*xkc...
           +(-39*sqrt(21)+231)*xk1...
           + Ts*((3*sqrt(21)+21)*xdotk - 16*sqrt(21)*xdotkc...
           + (3*sqrt(21)-21)*xdotk1))/686;
    u1 = (uk+ukc)/2;
    xdot1 = goatFullDynamicsWithConstraints(x1,u1,l);
    
    x2 = ((-39*sqrt(21)+231)*xk + 224*xkc...
           +(39*sqrt(21)+231)*xk1...
           + Ts*((-3*sqrt(21)+21)*xdotk + 16*sqrt(21)*xdotkc...
           + (-3*sqrt(21)-21)*xdotk1))/686;
    u2 = (ukc+uk1)/2;
    xdot2 = goatFullDynamicsWithConstraints(x2,u2,l);
    
    delk = (xk - xk1) + Ts*(xdotk+4*xdotkc+xdotk1)/6;
    del1 = ((32*sqrt(21)+180)*xk - 64*sqrt(21)*xkc+...
            +(32*sqrt(21)-180)*xk1...
            +Ts*((9+sqrt(21))*xdotk + 98*xdot1 + 64*xdotkc+...
            +(9-sqrt(21))*xk1))/360;
        
    del2 = ((-32*sqrt(21)+180)*xk + 64*sqrt(21)*xkc+...
            +(-32*sqrt(21)-180)*xk1...
            +Ts*((9-sqrt(21))*xdotk + 98*xdot2 + 64*xdotkc+...
            +(9+sqrt(21))*xk1))/360;
 
    ceq = [ceq;del0;delk;del1;del2];
end    
    ceq = [ceq;p(1:12,N)-xref];
%     ceq = real(ceq);

end
