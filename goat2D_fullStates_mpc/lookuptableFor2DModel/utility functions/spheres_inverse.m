function F = spheres_inverse(x,foot)
%used for inverse kinematics calculations
global leg

% k1 = [11.86*cos(theta(1))/2 + 3.31;
%                      0;
%      -11.86*sin(theta(1))/2];
%
% k2 = [-11.86*cos(theta(2))/4 - 3.31/2;
%        3.31*3^(1/2)/2 + 11.86*3^(1/2)*cos(theta(2))/4;
%       -11.86*sin(theta(2))/2];
%
% k3= [-11.86*cos(theta(3))/4 - 3.31/2;
%        -3.31*3^(1/2)/2 - 11.86*3^(1/2)*cos(theta(3))/4;
%       -11.86*sin(theta(3))/2];

F(1) = (foot(1) - (11.86*cos(x(1))/2 + 3.31))^2 + (foot(2) - 0)^2 +(foot(3) - (-11.86*sin(x(1))/2))^2 - (leg.l2)^2;
F(2) = (foot(1) - (-11.86*cos(x(2))/4 - 3.31/2))^2 + (foot(2) - (3.31*3^(1/2)/2 + 11.86*3^(1/2)*cos(x(2))/4))^2 +(foot(3) + 11.86*sin(x(2))/2)^2 - (leg.l2)^2;
F(3) = (foot(1) - (-11.86*cos(x(3))/4 - 3.31/2))^2 + (foot(2) - ( -3.31*3^(1/2)/2 - 11.86*3^(1/2)*cos(x(3))/4))^2 +(foot(3) + 11.86*sin(x(3))/2)^2 - (leg.l2)^2;
