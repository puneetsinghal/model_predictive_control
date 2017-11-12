function g = rigid_body_transformation(R, t)
%returns the rigid body transformation belonging to SE(3)
% Inputs:
  % R: rotation matrix
  % t: translations

g=zeros(4,4);
g(1:3, 1:3) = R;
g(1:4,4) = homogeneousRepOfPoint(t);


end
