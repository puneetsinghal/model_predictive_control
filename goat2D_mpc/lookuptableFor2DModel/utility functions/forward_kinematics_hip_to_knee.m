function [ k ] = forward_kinematics_hip_to_knee(leg, h ,theta)
% Inputs:
  % - h: the hip on which fwd kinematics is done
  % - leg
  % - theta: angle of joint
% outputs:
  % - k: position of the corresponding knee at an angle teta of the hip
  
k0 = [leg.l1; 0; 0];
k = expm( wedge( h.twist) * theta ) * h.g0 * homogeneousRepOfPoint(k0);
k=k(1:3);
end
