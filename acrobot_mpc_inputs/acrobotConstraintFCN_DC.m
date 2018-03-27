function [c, ceq] = acrobotConstraintFCN_DC(dynamics, u, x0, xref, params)
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

%     c = [];
    ceq = [];

    phiMin = [-pi;-pi];
    phiMax = [pi;pi];

    c = zeros(2*params.N, 2);
    xk1 = x0;
    for ct = 1:params.N
%         uk = u(ct);

        results = ode45(@(t,x)dynamics(t, x, u(:,ct), params), linspace(0, params.Ts), xk1);
        xk1 = results.y(:,end);

%         -phi + phiMin > 0
        c(2*ct-1,:) = xk1(1:2) - phiMin;
%         phi - phiMax > 0
        c(2*ct,:) = -xk1(1:2) + phiMax;

%         update plant state and input for next step
%         xk = xk1;
    end
%% equality constraints
%     N = params.N;
%     ceq = zeros(4,N+1);
%     ceq(:,1) = p(1:4,1) - x0;
%     for tk = 1:N-1
%         xk = p(1:4,tk);
%         uk = p(5,tk);
%         dxk = acrobotDynamicsCT(xk, uk, params);
% 
%         xk1 = p(1:4,tk+1);
%         uk1 = p(5,tk+1);
%         dxk1 = acrobotDynamicsCT(xk1, uk1, params);
% 
%         xc = (xk + xk1)/2 + params.Ts*(dxk - dxk1)/8;
%         uc = (uk+uk1)/2;
%         dxc = acrobotDynamicsCT(xc, uc, params);
% 
%         delk = (xk - xk1) + params.Ts*(dxk+4*dxc+dxk1)/6;
%         ceq(:,tk+1) = delk;
%     end
% 
%     ceq(:,end) = p(1:4,end)-xref;
end
