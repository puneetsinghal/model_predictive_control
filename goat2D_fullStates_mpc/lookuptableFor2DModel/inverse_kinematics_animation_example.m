%% Run these first!
% setup;
% load('InverseKinematics_lookupTable_US.mat');
%%
ViewZ=[linspace(-37.5,30,500).',...
           linspace(50,10,500).'];
k = 0;
angles=[];
 for i=1:1:500 %%%%%% Specify angles
     k=k+1;
%%%%%%%%%%%% 
%%Trajectory using Cartesian Coordinates
leg.foot = [8*cos(2*i*pi/100),0,-2*sin(2*i*pi/100)-12 ];
x = leg.foot(1);
y = leg.foot(2);
z = leg.foot(3);
foot = [ sqrt(x^2 + y^2 + z^2), rad2deg(atan2(y,x)),rad2deg(atan2(-z,sqrt(x^2+y^2)))];


%%trajectory using spherical coordinates
% foot = [-4*sin(2*i*pi/50) + 12,0,90];
% r = foot(1);
% theta = deg2rad(foot(2));
% phi = deg2rad(foot(3));
% leg.foot = [r*cos(phi)*cos(theta),r*cos(phi)*sin(theta), -r*sin(phi)];%given.. we want to find the corresponding joint angles

%%%%%%%%%%%%
% knees positions
%  fun = @spheres_inverse;% nonlinear function representing the three spheres of possbile foot poisitions from each knee
%  x0 = [0,0,0];
% theta= [theta; fsolve(@(x)fun(x,leg.foot),x0)];%,options
 
angles = get_joints( foot, inv_kmtcs)';
angles = deg2rad(angles);
leg.h1.k.p = forward_kinematics_hip_to_knee(leg, leg.h1,angles(1));
leg.h2.k.p = forward_kinematics_hip_to_knee(leg, leg.h2,angles(2));
leg.h3.k.p = forward_kinematics_hip_to_knee(leg, leg.h3,angles(3));

figure(1)
animation_goat();
% view(ViewZ(k,:));
F(k) = getframe;
% pause(0.00001);
 end
%%
% load('jumping2.mat')
movie(figure,F,1,30);
%% Save
save('jumping2.mat','F');
