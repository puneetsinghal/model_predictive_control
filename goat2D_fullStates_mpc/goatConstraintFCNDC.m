function [c, ceq] = goatConstraintFCNDC(p, x, xref, Ts, N, link_length)
%% Nonlinear MPC design parameters
% Range of theta posn
%Constarian Initial Point
ceq = p(1:12,1) - x;
for tk = 1:N-1   
    xk = [p(1:12,tk)];
    uk = p(13:14,tk);
    xdotk = goatFullDynamicsCT(xk,uk,link_length);
    
    xk1 = [p(1:12,tk+1)];
    uk1 = p(13:14,tk+1);
    xdotk1 = goatFullDynamicsCT(xk1,uk1,link_length);
    
    xkc = (xk+xk1)/2 + Ts*(xdotk - xdotk1)/8;
    ukc = (uk+uk1)/2;
    xdotkc = goatFullDynamicsCT(xkc,ukc,link_length);
    
    thetaeq = zeros(3,1); n=0;
    thetadoteq = zeros(3,1);
    % Impose theta and theta_dot constraints
    thetaeq(1) = link_length(1)*(cos(xkc(1))-cos(xkc(4)))...
                + link_length(2)*(cos(xkc(2))-cos(xkc(5)))...
                + link_length(3)*(cos(xkc(3))-cos(xkc(6)));
    thetaeq(2) = link_length(1)*(sin(xkc(1))-sin(xkc(4)))...
                + link_length(2)*(sin(xkc(2))-sin(xkc(5)))...
                + link_length(3)*(sin(xkc(3))-sin(xkc(6))); 
    thetaeq(3) = xkc(1) + xkc(2) + xkc(3) - xkc(4) - xkc(5) - xkc(6) - (2*n+1)*pi; % n depends on bounds on theta
    thetadoteq(1) = link_length(1)*(cos(xkc(1))*xkc(7)-cos(xkc(4))*xkc(10))...
                   + link_length(2)*(cos(xkc(2))*xkc(8)-cos(xkc(5))*xkc(11))...
                   + link_length(3)*(cos(xkc(3))*xkc(9)-cos(xkc(6))*xkc(12));
    thetadoteq(2) = link_length(1)*(sin(xkc(1))*xkc(7)-sin(xkc(4))*xkc(10))...
                   + link_length(2)*(sin(xkc(2))*xkc(8)-sin(xkc(5))*xkc(11))...
                   + link_length(3)*(sin(xkc(3))*xkc(9)-sin(xkc(6))*xkc(12));
    thetadoteq(3) = xkc(7) + xkc(8) + xkc(9) - xkc(10) - xkc(11) - xkc(12);
    
    %Defect
    delk = (xk - xk1) + Ts*(xdotk+4*xdotkc+xdotk1)/6;
    ceq = [ceq thetaeq' thetadoteq' delk];
end    
    %Constrain Final point
    ceq = [ceq [p(1:12,N)-xref]];
    ceq = real(ceq);

end
