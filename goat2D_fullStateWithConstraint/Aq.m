function A = Aq(theta,l)

A = zeros(3,6);
A(1,1) = l(1)*sin(theta(1)) + l(2)*sin(theta(1)+theta(2)) + l(3)*sin(theta(1)+theta(2)+theta(3));
A(1,2) = l(2)*sin(theta(1)+theta(2)) + l(3)*sin(theta(1)+theta(2)+theta(3));
A(1,3) = l(3)*sin(theta(1)+theta(2)+theta(3));
A(1,4) = -(l(1)*sin(theta(4)) + l(2)*sin(theta(4)+theta(5)) + l(3)*sin(theta(4)+theta(5)+theta(6)));
A(1,5) = -(l(2)*sin(theta(4)+theta(5)) + l(3)*sin(theta(4)+theta(5)+theta(6)));
A(1,6) = -l(3)*sin(theta(4)+theta(5)+theta(6));

A(2,1) = l(1)*cos(theta(1)) + l(2)*cos(theta(1)+theta(2)) + l(3)*cos(theta(1)+theta(2)+theta(3));
A(2,2) = l(2)*cos(theta(1)+theta(2)) + l(3)*cos(theta(1)+theta(2)+theta(3));
A(2,3) = l(3)*cos(theta(1)+theta(2)+theta(3));
A(2,4) = -(l(1)*cos(theta(4)) + l(2)*cos(theta(4)+theta(5)) + l(3)*cos(theta(4)+theta(5)+theta(6)));
A(2,5) = -(l(2)*cos(theta(4)+theta(5)) + l(3)*cos(theta(4)+theta(5)+theta(6)));
A(2,6) = -l(3)*cos(theta(4)+theta(5)+theta(6));

A(3,:) = [1,1,1,-1,-1,-1];

end