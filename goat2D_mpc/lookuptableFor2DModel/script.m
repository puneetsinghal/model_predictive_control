% By: Hadi Salman
% Date: 09/27/2016
global leg
setup();
%% hip 1, 2, and 3 of the leg

% [leg.h1, leg.h2] = goat_initialize();


%% ONE CONFIGURATION TESTING for fwd kinematics
%  tic
theta = ones(2,1);
theta(1)=-0.0;
theta(2)=-0.0;
% fwd kinematics
leg.foot= joint2foot(theta);
          
figure(1)
animation_goat();
%% DERIVATION FOR SYMBOLIC EQUATIONS OF POSITIONS OF knees k1, k2, and k3 USED FOR INVERSE kinematics
%
%   syms teta
%   k1  = forward_kinematics_hip_to_knee(leg, leg.h1 ,teta);% symbolic calculation of poistion of k1 as a funciton of joint angle theta1
% V = double(subs(k1,teta,deg2rad(-45))); % testing

%  k2  = forward_kinematics_hip_to_knee(leg, leg.h2 ,teta);% symbolic calculation of poistion of k2 as a funciton of joint angle theta2
% V = double(subs(k2,teta,deg2rad(-45))); % testing

% k3  = forward_kinematics_hip_to_knee(leg, leg.h3 ,teta);% symbolic calculation of poistion of k3 as a funciton of joint angle theta3
% V = double(subs(k3,teta,deg2rad(-45))); % testing

% k1 = simplify(k1,'Step', 100);
% k2 = simplify(k2,'Step', 500);
% k3 = simplify(k3,'Step', 100);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Results
% theta= [0,0,0];
% k1 = [11.86*cos(theta(1)) + 3.31;
%                      0;
%      -11.86*sin(theta(1))];
%
% k2 = [-11.86*cos(theta(2))/4 - 3.31/2;
%        3.31*3^(1/2)/2 + 11.86*3^(1/2)*cos(theta(2))/4;
%       -11.86*sin(theta(2))/2];
%
% k3= [-11.86*cos(theta(3))/4 - 3.31/2;
%        -3.31*3^(1/2)/2 - 11.86*3^(1/2)*cos(theta(3))/4;
%       -11.86*sin(theta(3))/2];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% ONE CONFIGURATION TESTING for inverse kinematics (want joint angles given a foot poisition)
r = 12;
theta = 0;
phi = pi/2;

foot = [r*cos(phi)*cos(theta),r*cos(phi)*sin(theta), -r*sin(phi)];%given.. we want to find the corresponding joint angles

% foot = [7.5, 0, 0];%given.. we want to find the corresponding joint angles

% knees' positions in terms of joint angles are the above in the Results section derived symbolically

% I will use instead of Optimization, a numerical nonlinear equations solver
fun = @spheres_inverse;% nonlinear function representing the three spheres of possbile foot poisitions from each knee
x0 = [0,0,0];
% fun(x0,foot)
% options = optimoptions('fsolve','Display','iter');
joints= fsolve(@(x)fun(x,foot),x0);%,options
    toc
rad2deg(joints)



%% Forward Kinematics: given input joint angles, returns output: foot position
% This creates the lookup table
N = 100;%number of intervals in discretization(resolution)
fwd_kmtcs = cell(N+1,N+1);
for theta1 = -pi/2 : pi/N : pi/2

    index1 = uint8((theta1 + pi/2)*N/pi+1); %index of the first joint in the cell

    for theta2 = -pi/2 : pi/N : pi/2

        index2 = uint8((theta2 + pi/2)*N/pi+1); %index of the second joint in the cell array

            %%Computing knee positions at that configuration
            leg.h1.k.p = forward_kinematics_hip_to_knee(leg, leg.h1,theta1);
            leg.h2.k.p = forward_kinematics_hip_to_knee(leg, leg.h2,theta2);
              fun = @spheres_fwd;% nonlinear function representing the three spheres of possbile foot poisitions from each knee
              x0 = [0,0,-200];
%               options = optimoptions('fsolve','Display','iter');
              foot= fsolve(fun,x0);%,options

              fwd_kmtcs {index1, index2} = foot(:);
    end
end

save('ForwardKinematics_2D.mat','fwd_kmtcs');




%% Inverse Kinematics: given foot position, returns the input  joint angles
N = 100;%number of intervals in discretization(resolution)
inv_kmtcs = cell(N+1,N+1,N+1);
teta = zeros(3,1);
% load('ForwardKinematics_US.mat');
for r = 7.5 : (18-7.5)/N : 18

    index1 = uint8((r-7.5)*N/(18-7.5)+1); %index of the first joint in the cell

    for theta = 0 : 2*pi/N : 2*pi

        index2 = uint8((theta)*N/(2*pi)+1); %index of the second joint in the cell array

        for phi = 0 : pi/(2*N) : pi/2

            index3 = uint8((phi)*2*N/pi+1); %index of the third joint in the cell array
            
            foot = [r*cos(phi)*cos(theta),r*cos(phi)*sin(theta), -r*sin(phi)];%given.. we want to find the corresponding joint angles

            fun = @spheres_inverse;% nonlinear function representing the three spheres of possbile foot poisitions from each knee
            x0 = [0,0,0];
            % fun(x0,foot)
            % options = optimoptions('fsolve','Display','iter');
            joints= fsolve(@(x)fun(x,foot),x0);%,options
                
            teta(1)= rad2deg(joints(1));%[deg]
            teta(2)= rad2deg(joints(2));%[deg]     
            teta(3)= rad2deg(joints(3));%[deg]
            
            if(all(teta<90) && all(teta>-90) && norm(joint2foot(teta)-foot')<0.001)
            inv_kmtcs {index1, index2, index3} = teta(:);
            else
            inv_kmtcs {index1, index2, index3} = nan;
            end
        end
    end
end

 save('InverseKinematics.mat','inv_kmtcs');


%% Jacobian Calculation
% call Jacobian_lookupTable_calculation once --> Jacobian_lookup_table_US.mat
% is saved ---> contains a variable called Jacobian (100x100x100 cells containging the jacobians at discretized joint angles)
%%%%%%%%%%%%%%%%%%%
 load ('Jacobian_lookupTable_US.mat') % uncomment this and run it once
%%%%%%%%%%%%%%%%%%%
% call return jacobian to get jacobian at any set of hip angles between [-pi/2, pi/2]
hip_angles = [0,0,0];
jac = get_jacobian(hip_angles, Jacobian);

%% get joint angles given foot position from lookup table ()
% specify foot as (r, theta, phi)
% load('InverseKinematics_lookupTable_US.mat');

foot = [10.25, 0, 90];
get_joints(foot,inv_kmtcs)

%% get foot positions
