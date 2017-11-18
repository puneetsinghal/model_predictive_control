function [c, ceq] = goatConstraintFCNDC(p, x, xref, Ts, N, link_length)
%% Nonlinear MPC design parameters
% Range of theta posn
%Constarian Initial Point
ceq = p(1:6,1) - x;
for tk = 1:N-1   
%     tk
    xk = [p(1:6,tk)];
    uk = p(7:8,tk);
    xdotk = goatDynamicsCT(xk,uk,link_length);
    
    xk1 = [p(1:6,tk+1)];
    uk1 = p(7:8,tk+1);
    xdotk1 = goatDynamicsCT(xk1,uk1,link_length);
    
    xkc = (xk+xk1)/2 + Ts*(xdotk - xdotk1)/8;
    ukc = (uk+uk1)/2;
    xdotkc = goatDynamicsCT(xkc,ukc,link_length);
    %Defect
    delk = (xk - xk1) + Ts*(xdotk+4*xdotkc+xdotk1)/6;
    ceq = [ceq delk];
end    
    %Constrain Final point
    ceq = [ceq [p(1:6,N)-xref]];
    ceq = real(ceq);

end
