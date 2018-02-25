function [c, ceq] = constraintFCN(u, x, trajRef, time, params)
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
% zMin = -10;
% zMax = 10;

%% Inequality constraints calculation
c = [];
ceq = [];
% Apply 2*N cart position constraints across prediction horizon, from time
% k+1 to k+N
xk = x;
uk = u(:,1);
N = params.N;
% traj = zeros(size(trajDesired));
trajPoint = 1;
for ct=1:time(end)/params.Ts
    % obtain new cart position at next prediction step
    
    xk1 = acrobotDynamicsDT(xk, uk, params);
    % -z + zMin < 0
%     c(2*ct-1) = -xk1(1)+zMin;
    % z - zMax < 0
%     c(2*ct) = xk1(1)-zMax;
    % update plant state and input for next step
    xk = xk1;
    if (ct*params.Ts == time(trajPoint))
%         traj(:,ct) = xk1;
        trajPoint = trajPoint + 1;
        if (norm(xk1(1:2) - trajRef(1:2, trajPoint)) > 0.001)
            ceq = 10;
        end
    end
    
    if ct<N
        uk = u(:,ct+1);
    end
end

% c = 
%% No equality constraints
% if (norm(xk(1:4) - xref(1:4)) > 0.005)
%     [xk(1:4); xref(1:4)]'
%     ceq = 1
% else
%     disp('good');
% end
    
