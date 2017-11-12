function [ w ] = omegaFromTwist( T,N,n )
%returns the angular velocity(matrix) vector given the corresponding twist
%vector(matrix)
w=zeros(3,N,n);
w(:,:,:)=T(4:6,:,:);


end
