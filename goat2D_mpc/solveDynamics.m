%% Initital configuration of goat
% Vertically upward position can be found using the function "findVerticalInitialConfiguration"
% Output of the function is in format: [theta1(1), theta1(2), theta1(3), theta2(1), theta2(2), theta2(3)]
% One feasible solution calculated using the same function is:
% [2.0634, -1.0701, -0.9933, 1.0781, 1.0701, 0.9933]
addpath('lib')
%%

% link lengths are defined as global variable: global_link_length
% We need this function inside dynamics.m and forwardKinematcs.m and it is easier to use global
% variable then to pass it as arguments everytime

global global_link_length
global_link_length = [1.0 0.5 0.2];

% This section is to initialize the animation file to see the live motion.
% Later, we are going to replay everything which gives a smooth motion and
% user can choose to create video file if the results are good :)

global hlink hjnt
figure;
clf
axes
view(2)
axis image
hold on
axis([-sum(global_link_length)*1.2, sum(global_link_length)*1,...
    -sum(global_link_length)*1.2, sum(global_link_length)*1]);

pose = forwardKinematics([2.0634, -1.0701, -0.9933, 1.0781, 1.0701, 0.9933], global_link_length);
% format of pose = [[0;0], g11(1:2,3), g12(1:2,3), g13(1:2,3), [0;0], g21(1:2,3), g22(1:2,3), g23(1:2,3)];

for i =1:length(global_link_length)
    hlink(i) = plot([pose(1,i); pose(1,i+1)], [pose(2,i); pose(2,i+1)], 'Color', [0 0 0.7], 'LineWidth', 5);
    hjnt(i) = plot(pose(1,i),pose(2,i),'.', 'MarkerSize', 20, 'Color', [1 0 0]);
end

for i =4:2*length(global_link_length)
    hlink(i) = plot([pose(1,i+1); pose(1,i+2)], [pose(2,i+1); pose(2,i+2)], 'Color', [0 0 0.7], 'LineWidth', 5);
    hjnt(i) = plot(pose(1,i+2),pose(2,i+2),'.', 'MarkerSize', 20, 'Color', [1 0 0]);
end
shg
pause(1); % this pause is given to allow time to show the initial configuration
%% Initialization
%  We are using ODE45 to run the simulation by using
% dynamics(t, q, input, q10) function handle


% u0 = [0; 0]; % to test free fall dynamics
u0 = 1.6064248*[120; -120];

q20 = [1.0781, 1.0701, 0.9933]';
dq20 = [0, 0, 0]';
q10 = [2.0634, -1.0701, -0.9933]';

q = [q10,q20];
%% Linearization
[B, K] = linearizationAndLQR(q10, q20, dq20, u0);

%%
xNominal = [q20; dq20];
q2_initial = [q20; dq20] + [+0.012; 0; 0; 0; 0; 0];

drawSwitch = 1;

[t,qs, q1] = ode45(@(t,q)dynamics(t, q, u0, q10, xNominal, K, B, drawSwitch), [0 , 4], q2_initial);
save('results.mat','t','qs', 'q10');
%% PLAYBACK
if(~drawSwitch)
    load('results.mat');
    playback(t, qs, q10);
end
