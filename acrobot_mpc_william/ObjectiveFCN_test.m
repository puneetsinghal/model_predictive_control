%	ObjectiveFCN_test(u, x0, xref, u0, params)
% N_s - size of state space, N_a - size of action space, M - size of horizon
% u: try to minimize J = J(u). the optimal control usage, shape(N_a,M)
% x0: initial state starting from, shape(N_s,1)
% xref: target trajectory, shape(N_s,M)
% M is the size of the receeding horizons
% u0: usage of previos time step, size(N_a,1)
% params: system parameters
% Performance weight
function J = ObjectiveFCN_test(u, u0, x0, xref, params, dynamics)
		N_s = size(x0,1);
		N_a = size(u0,1);
		M = size(xref,2);
		
		% Performance weight
		Q = diag([100; 100; 0.01; 0.01]); %[10000*eye(4), zeros(4);zeros(4), 0.01*eye(4)];
    % Usage weight
		R = 0.01;
		
    J = 0;
    
		x_prev = x0;
		u_prev = u0;

    for i = 1:M
        u_k = u(:,i);
				xref_k = xref(:,i);
				
				% Find the current state
        %x_k = acrobotDynamicsDT(x_prev, u_prev, params);
        x_k = ode45(@(t,x)dynamics(t,x,u_prev,params), [0,params.Ts], x_prev);
        %x_k = ode45(@(t,y)dynamics(t,y,params), [0,params.Ts], x_prev)
				x_k = x_k.y(:,end);
				J = J + (x_k-xref_k)'*Q*(x_k-xref_k) + (u_k - u_prev)'*R*(u_k - u_prev);
			
				u_prev = u_k;
				x_prev = x_k;	
    end    
end


