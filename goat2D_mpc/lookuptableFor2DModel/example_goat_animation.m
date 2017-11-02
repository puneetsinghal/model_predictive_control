%% jumping gait
load ('jumping.mat');
fprintf('[Jumping gait]...\n')
movie(figure(1),F,1,70);
movie2avi(F,'jumping.avi')

%% Rotate in x-y plane gait
load ('demo_rotate_gait.mat');
fprintf('[Rotate in x-y plane]...\n')
movie(figure(1),F,1,70);

%% Forward Gait
load ('demo_forward_gait.mat');
fprintf('[Forward Gait]...\n')
movie(figure(1),F,1,40);
% movie2avi(F,'fwdGait.avi')

%% Back and Forward Gait
load ('demo_back&for_gait.mat');
fprintf('[back and for Gait]...\n')
movie(figure(1),F,1,60);
movie2avi(F,'fwd&backGait.avi')
%% OR CHOOSE YOUR OWN INPUT ANGLES TO ANIMATE (uncomment all first!)
setup;
load('ForwardKinematics_US.mat')

%%
ViewZ=[linspace(-37.5,30,1300).',...
           linspace(50,10,1300).'];
k = 0;
for l=1:3
for j=1:-2:-1     %%%%%% up then down
 for i=-45:0.5:45 %%%%%% Specify angles
     k=k+1;
%%%%%%%%%%%% 
     teta=i*j;
theta=([teta, teta, teta]);
%%%%%%%%%%%
% knees positions
leg.foot= joint2foot(theta);%,options

% figure(1)
animation_goat();
view(ViewZ(k,:));
 F(k) = getframe;
% pause(0.00001);
end
end
end
% movie(F,1,200)
%% Save
save('demo_using_lookupTable.mat','F');
