function [ M ] = wedge( v )
%returns the matrix in se(3) associated with a vector in R6 (twist coordinates)

M=zeros(4,4);
M(1:3,1:3)=vector2HatRep(v(4:6));
M(1:3,4)=v(1:3);


end
