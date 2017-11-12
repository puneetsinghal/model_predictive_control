function F = spheres_fwd(x)
%used for forward kinematics calculations
global leg
F(1) = (x(1) - leg.h1.k.p(1))^2 + (x(2) - leg.h1.k.p(2))^2 +(x(3) - leg.h1.k.p(3))^2 - (leg.l2)^2;
F(2) = (x(1) - leg.h2.k.p(1))^2 + (x(2) - leg.h2.k.p(2))^2 +(x(3) - leg.h2.k.p(3))^2 - (leg.l2)^2;
F(3) = x(2)^2;
