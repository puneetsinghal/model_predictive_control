function [c, ceq] = goatConstraintFCNDC(p, x, xref, Ts, N, link_length)
%% Constraint function of nonlinear MPC for pendulum swing-up and balancing control
%
% Inputs:
%   u:      optimization variable, from time k to time k+N-1 
%   x:      current state at time k
%   Ts:     controller sample time
%   N:      prediction horizon
%
% Output:
%   c:      inequality constraints applied across prediction horizon
%   ceq:    equality constraints (empty)
%
% Copyright 2016 The MathWorks, Inc.

%% Nonlinear MPC design parameters
% Range of cart position: from -10 to 10
zMin = [0; 0; 0; pi/2; -pi/2; -pi/2];
zMax = [pi/2; pi; pi/2; pi; 0; 0];

%% Inequality constraints calculation
c = [];
% c = zeros(12,N);
% % Apply 2*N cart position constraints across prediction horizon, from time
% % k+1 to k+N
% xk = x;
% uk = u(:,1);
% for ct=1:N
%     xk1 = goatDynamicsDT(xk, uk, Ts, link_length);
%     xk2 = findFeasibleConfigurationAnalytical(xk1, link_length);
% %     c(1:6,ct) = -[xk1(1:3);xk2] + zMin;
% %     c(7:12,ct) =  [xk1(1:3);xk2] - zMax;
% %     c(1:6,ct) = -[xk1(1:3);xk1(7:9)] + zMin;
% %     c(7:12,ct) =  [xk1(1:3);xk1(7:9)] - zMax;
% %     
%      % update plant state and input for next step
%     xk = xk1;
%     
%     if ct<N
%         uk = u(:,ct+1);
%     end
% end
%% Equality constraints
%Constarian Initial Point
ceq = p(1:6,1) - x;
for tk = 1:N-1   
    
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
