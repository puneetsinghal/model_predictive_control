function [ Ad ] = Adjoint( g )
%returns the adjoint transfromation 6x6 matrix associated with a rigid body transformation g (SE(3)) 

Ad=zeros(6,6);

[R,P]=RigidTransComponents(g);
Ad(1:3,1:3)=R;
Ad(4:6,4:6)=R;
Ad(1:3,4:6)=vector2HatRep(P)*R;

end

