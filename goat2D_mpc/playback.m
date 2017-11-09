function playback(t, q2_trajectory, q10)

%% PLAYBACK

% link lengths are defined as global variable: global_link_length
% We need this function inside dynamics.m and forwardKinematcs.m and it is easier to use global
% variable then to pass it as arguments everytime

global global_link_length;

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


%%
for j = 1:2:length(t)
    q2 = q2_trajectory(j, 1:3)';
    
%     q1 = fsolve(@(q1) constraints(q1,q2), q10);
%     q1 = wrapToPi(q1);
    q1 = findFeasibleConfigurationAnalytical(q2, global_link_length);
    pose = forwardKinematics([q1; q2], global_link_length);
    for i =1:length(global_link_length)
        set(hlink(i),...
            'XData', [pose(1,i); pose(1,i+1)],...
            'YData', [pose(2,i); pose(2,i+1)]);
        set(hjnt(i), 'XData', pose(1,i),'YData', pose(2,i));
    end
    for i =4:2*length(global_link_length)
        set(hlink(i),...
            'XData', [pose(1,i+1); pose(1,i+2)],...
            'YData', [pose(2,i+1); pose(2,i+2)]);
        set(hjnt(i), 'XData', pose(1,i+2),'YData', pose(2,i+2));
    end 
    drawnow
end
end