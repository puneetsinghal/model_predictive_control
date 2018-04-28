function [c, ceq] = acrobotConstraintFCN_DC(p, x0, xref, params)
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

c = [];

%% equality constraints
    N = params.N;
    ceq = zeros(6,N+1);
    ceq(:,1) = p(1:6,1) - x0;
    for tk = 1:N-1
        xk = p(1:6,tk);
        uk = p(7:end,tk);
        dxk = acrobotDynamicsCT(xk, uk, params);

        xk1 = p(1:6,tk+1);
        uk1 = p(7:end,tk+1);
        dxk1 = acrobotDynamicsCT(xk1, uk1, params);

        xc = (xk + xk1)/2 + params.Ts*(dxk - dxk1)/8;
        uc = (uk+uk1)/2;
        dxc = acrobotDynamicsCT(xc, uc, params);

        delk = (xk - xk1) + params.Ts*(dxk+4*dxc+dxk1)/6;
        ceq(:,tk+1) = delk;
    end

    ceq(:,end) = p(1:6,end)-xref;
end