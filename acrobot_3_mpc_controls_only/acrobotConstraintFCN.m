function [c, ceq] = acrobotConstraintFCN(p, x0, xref, params)
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

% c = [];

%% equality constraints
    N = params.N;
    xk = x0;
    LB = [-2*pi*ones(3,1); -100*ones(3,1)];
    UB = [ 2*pi*ones(3,1); 100*ones(3,1)];
    c = zeros(6, 2*N);
    for tk = 1:N
        uk = p(tk);
        xk = acrobotDynamicsDT(xk, uk, params);
        c(:, 2*tk-1) = xk - LB;
        c(:, 2*tk) = xk + UB;
    end
    ceq = [];
end
