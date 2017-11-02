function [R,P] = RigidTransComponents( g )
%returns the rotation matrix and postion vector of a rigid body
%transformation SE(3)

R=g(1:3,1:3);
P=g(1:3,4);

end
