function [ M ] = vector2HatRep( v )
%convert a vector to it's corresponding skew symmetric matrix that converts cross product into matrix-vector multiplication 
 
M=[0 -v(3) v(2);v(3) 0 -v(1); -v(2) v(1) 0];

end

