function [c, ceq] = goatConstraintFCNDC(p, x, xref, Ts, N, link_length)
%% Nonlinear MPC design parameters
% Range of theta posn
%Constarian Initial Point
c = [];
ceq = p(1:12,1) - x;
thetaeq = zeros(3,1); n=0;
thetadoteq = zeros(3,1);
% Impose theta and theta_dot constraints
thetaeq(1) = link_length(1)*(cos(p(1,1))-cos(p(4,1)))...
            + link_length(2)*(cos(p(2,1))-cos(p(5,1)))...
            + link_length(3)*(cos(p(3,1))-cos(p(6,1)));
thetaeq(2) = link_length(1)*(sin(p(1,1))-sin(p(4,1)))...
            + link_length(2)*(sin(p(2,1))-sin(p(5,1)))...
            + link_length(3)*(sin(p(3,1))-sin(p(6,1))); 
thetaeq(3) = p(1,1) + p(2,1) + p(3,1) - p(4,1) - p(5,1) - p(6,1) - (2*n+1)*pi; % n depends on bounds on theta
thetadoteq(1) = link_length(1)*(cos(p(1,1))*p(7,1)-cos(p(4,1))*p(10,1))...
               + link_length(2)*(cos(p(2,1))*p(8,1)-cos(p(5,1))*p(11,1))...
               + link_length(3)*(cos(p(3,1))*p(9,1)-cos(p(6,1))*p(12,1));
thetadoteq(2) = link_length(1)*(sin(p(1,1))*p(7,1)-sin(p(4,1))*p(10,1))...
               + link_length(2)*(sin(p(2,1))*p(8,1)-sin(p(5,1))*p(11,1))...
               + link_length(3)*(sin(p(3,1))*p(9,1)-sin(p(6,1))*p(12,1));
thetadoteq(3) = p(7,1) + p(8,1) + p(9,1) - p(10,1) - p(11,1) - p(12,1);
ceq = [thetaeq;thetadoteq;ceq];
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
    ceq = [ceq;thetaeq;thetadoteq;delk];
end    
    %Constrain Final point
    thetaeq = zeros(3,1); n=0;
    thetadoteq = zeros(3,1);
    % Impose theta and theta_dot constraints
    thetaeq(1) = link_length(1)*(cos(p(1,1))-cos(p(4,1)))...
                + link_length(2)*(cos(p(2,1))-cos(p(5,1)))...
                + link_length(3)*(cos(p(3,1))-cos(p(6,1)));
    thetaeq(2) = link_length(1)*(sin(p(1,1))-sin(p(4,1)))...
                + link_length(2)*(sin(p(2,1))-sin(p(5,1)))...
                + link_length(3)*(sin(p(3,1))-sin(p(6,1))); 
    thetaeq(3) = p(1,N) + p(2,N) + p(3,N) - p(4,N) - p(5,N) - p(6,N) - (2*n+1)*pi; % n depends on bounds on theta
    thetadoteq(1) = link_length(1)*(cos(p(1,N))*p(7,N)-cos(p(4,N))*p(10,N))...
                   + link_length(2)*(cos(p(2,N))*p(8,N)-cos(p(5,N))*p(11,N))...
                   + link_length(3)*(cos(p(3,N))*p(9,N)-cos(p(6,N))*p(12,N));
    thetadoteq(2) = link_length(1)*(sin(p(1,N))*p(7,N)-sin(p(4,N))*p(10,N))...
                   + link_length(2)*(sin(p(2,N))*p(8,N)-sin(p(5,N))*p(11,N))...
                   + link_length(3)*(sin(p(3,N))*p(9,N)-sin(p(6,N))*p(12,N));
    thetadoteq(3) = p(7,N) + p(8,N) + p(9,N) - p(10,N) - p(11,N) - p(12,N);
    ceq = [ceq;thetaeq;thetadoteq;p(1:12,N)-xref];
%     ceq = real(ceq);

end
