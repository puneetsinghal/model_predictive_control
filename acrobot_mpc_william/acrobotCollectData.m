clear all;
close all;
clc;

% load the configuration
param_config;

% Initial condition
x0 = [2.0;0.1;0.1;0.1];

% Contol input
u = 0;
x_prev = x0;

% Total data points to collect
T = 10000;
%	Input column: [sita0, sita1, w0, w1, tau]
% Output columt: [sita0, sita1, w0, w1] (t+1)
Input_data = zeros(5,T);
Output_data = zeros(4,T);

% Set the reasonable range of input data




render = 0;




% Collect 
for i = 1:T
	% Random state input
	
	if mod(i,1000) == 0
		sprintf('%d',i);
	end
	x_prev = rand_state();
	u = rand_input();

	Input_data(1:4,i) = x_prev;
	Input_data(5,i) = u;

	x_k = ode45(@(t,x)acrobotDynamicsCT(t,x,u,params),[0, params.Ts], x_prev);
	x_k = x_k.y(:,end);
	
	Output_data(:,i) = x_k;
	if render
		drawAcrobot(i*params.Ts, x_k, params);
		pause(0.01);
	end
	
	%x_prev = x_k;

end

Input_data = Input_data';
Output_data = Output_data';

save('data.mat','Input_data','Output_data');



% Return a random input data within a reasonable range
function x_rand = rand_state()
	sita_min = -pi;
	sita_max = pi;
	w_min = -20;
	w_max = 20;
	
	sita1 = sita_min + (sita_max - sita_min)*rand();
	sita2 = sita_min + (sita_max - sita_min)*rand();
	w1 = w_min + (w_max - w_min)*rand();	
	w2 = w_min + (w_max - w_min)*rand();	

	x_rand = [sita1;sita2;w1;w2];
end

% Return a random control input within a reasonable range
function u_rand = rand_input()
	u_min = -20;
	u_max = 20;
	u_rand = u_min + (u_max - u_min)*rand();
end


