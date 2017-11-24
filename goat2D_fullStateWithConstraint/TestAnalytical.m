%% Check Dynamics 
close all;
clear;
clc;

global global_link_length
global_link_length = [1.0 0.5 0.2];

global hlink hjnt
figure;
clf
axes
view(2)
axis image
hold on
axis([-sum(global_link_length)*1.2, sum(global_link_length)*1,...
    -sum(global_link_length)*1.2, sum(global_link_length)*1]);
% angles = [1.0901    1.4201    0.2333    1.4190    1.3256   -2.3465];
angles = [1.0781, 1.0701, 0.993];
% angles = [2.0634, -1.0701, -0.9933, 1.0781, 1.0701, 0.9933];
% angles = [2.0987   -0.9260   -1.5708    1.1958    1.0374    1.3065];
% pose = forwardKinematics(angles, global_link_length);
a = findFeasibleConfigurationAnalytical(angles(1:3),global_link_length);
pose = forwardKinematicsAnalytical([a;angles'],global_link_length);
for i =1:length(global_link_length)
    hlink(i) = plot([pose(1,i); pose(1,i+1)], [pose(2,i); pose(2,i+1)], 'Color', [0 0 0.7], 'LineWidth', 5);
    hjnt(i) = plot(pose(1,i),pose(2,i),'.', 'MarkerSize', 20, 'Color', [1 0 0]);
end

for i =4:2*length(global_link_length)
    hlink(i) = plot([pose(1,i+1); pose(1,i+2)], [pose(2,i+1); pose(2,i+2)], 'Color', [0 0 0.7], 'LineWidth', 5);
    hjnt(i) = plot(pose(1,i+2),pose(2,i+2),'.', 'MarkerSize', 20, 'Color', [1 0 0]);
end
