function q  = homogeneousRepOfPoint( p )
%returns the homogenuous representation of a point (don't matter if input row or vector)
%output is a column vector
if(size(p,2) == 1)
  q=[p; 1];
else
  q=[p, 1];
end

% ensure output is column vector
q=q(:);

end
