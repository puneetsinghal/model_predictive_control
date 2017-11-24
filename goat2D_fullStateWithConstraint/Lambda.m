function L = Lambda(theta,l)
L = zeros(6,3);
% Lx, Ly, Lo
L(1,1) = -(l(3)*sin(theta(1)+theta(2)+theta(3))+l(2)*sin(theta(1)+theta(2))+l(1)*sin(theta(1)));
L(1,2) = (l(3)*cos(theta(1)+theta(2)+theta(3))+l(2)*cos(theta(1)+theta(2))+l(1)*cos(theta(1)));
L(1,3) = 1;
L(2,1) = -(l(3)*sin(theta(1)+theta(2)+theta(3))+l(2)*sin(theta(1)+theta(2)));
L(2,2) = (l(3)*cos(theta(1)+theta(2)+theta(3))+l(2)*cos(theta(1)+theta(2)));
L(2,3) = 1;
L(3,1) = -(l(3)*sin(theta(1)+theta(2)+theta(3)));
L(3,2) = l(3)*cos(theta(1)+theta(2)+theta(3));
L(3,3) = 1;

L(4,1) = (l(3)*sin(theta(1)+theta(2)+theta(3))+l(2)*sin(theta(1)+theta(2))+l(1)*sin(theta(1)));
L(4,2) = -(l(3)*cos(theta(1)+theta(2)+theta(3))+l(2)*cos(theta(1)+theta(2))+l(1)*cos(theta(1)));
L(4,3) = -1;
L(5,1) = (l(3)*sin(theta(1)+theta(2)+theta(3))+l(2)*sin(theta(1)+theta(2)));
L(5,2) = -(l(3)*cos(theta(1)+theta(2)+theta(3))+l(2)*cos(theta(1)+theta(2)));
L(5,3) = -1;
L(6,1) = (l(3)*sin(theta(1)+theta(2)+theta(3)));
L(6,2) = -l(3)*cos(theta(1)+theta(2)+theta(3));
L(6,3) = -1;
end