function [c, ceq] = constraintFCN(u, x, xref, Ts, N, global_link_length)
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
c = zeros(12,N);
% Apply 2*N cart position constraints across prediction horizon, from time
% k+1 to k+N
xk = x;
uk = u(:,1);
for ct=1:N
    xk1 = goatDynamicsDT(xk, uk, Ts, global_link_length);
   
    c(1:6,ct) = -[xk1(1:3);xk1(7:9)] + zMin;
    c(7:12,ct) =  [xk1(1:3);xk1(7:9)] - zMax;
    
     % update plant state and input for next step
    xk = xk1;
    
    if ct<N
        uk = u(:,ct+1);
    end
end

% c = 
%% No equality constraints
ceq = [];
if (norm(xk(1:6) - xref) > 0.001)
    ceq = 10000;
end
