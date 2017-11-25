function [c, ceq] = goatConstraintFCNDC(p, x, xref, Ts, N, l)
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
    
    xkc = (xk+xk1)/2 + Ts*(xdotk - xdotk1)/8;
    ukc = (uk+uk1)/2;
    xdotkc = goatFullDynamicsWithConstraints(xkc,ukc,l);
    %Defect
    delk = (xk - xk1) + Ts*(xdotk+4*xdotkc+xdotk1)/6;
    ceq = [ceq;delk];
end    
    ceq = [ceq;p(1:12,N)-xref];
%     ceq = real(ceq);

end
