function [h1, h2, h3 ] = goat_initialize()
%Initialize of the hip morphology
addpath('utility functions');

r = 3.31; %[inch]

% Positions of hip joints (hi)
h1.p= [r; 0 ; 0]; %[inch inch inch]
h2.p = [-r; 0;0]; %[inch inch inch]

% Directions of axes of hip joints (the outer minus specifies if positive joint angles is upward or downward)
h1.w = [0; 1; 0]; 
h2.w = [0; -1;0];

%% Twist coordinates
h1.twist=[cross(h1.p, h1.w); h1.w];
h2.twist=[cross(h2.p, h2.w); h2.w];


%% home configurations of the hip joints
h1.g0 = rigid_body_transformation(eye(3), [r; 0; 0]);%[cm cm cm]
h2.g0 = rigid_body_transformation([-1,0,0;0,-1,0;0,0,1], [-r; 0; 0]);%[cm cm cm]
%rotz take angles in degrees! Trigonnometric functions take angles in radians!
end
