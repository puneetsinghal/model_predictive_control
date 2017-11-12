%% this demo shows examples on how to use the lookup tables (inv_kmtcs and Jacobian)
% first laod the two lookup tables (only load once in your program)
load('ForwardKinematics_US.mat');% a 101x101x101 cell array "inv_kmtcs" well appear in the workspace
load('InverseKinematics_lookupTable_US.mat');% a 101x101x101 cell array "inv_kmtcs" well appear in the workspace
load('Jacobian_lookupTable_US.mat');% a 100x100x100 cell array "Jacobian" will appear in the workspace


%% get foot position given hip input angles
theta = [45,45,45];%[deg deg deg]
foot = get_foot_position(theta, fwd_kmtcs); %3 dimensional row vector [x,y,z]

%% extract jacobian matrix

theta = [0, 0, 0];% [deg deg deg]
J = get_jacobian(theta, Jacobian); % 3x3 matrix that relates [Fx;Fy;Fz] of the foot to [T1, T2, T3] the three motor torques 

%% extract hip joint angles given a foot position
% foot = [r, theta, phi] "spherical coordinates":    7.5 < r < 18  ////  0 < theta < 360  //// 0 < phi < 90

foot = [15.7752, 0, 90]; % [inch deg deg]
joints = get_joints(foot, inv_kmtcs); %[deg deg deg]
