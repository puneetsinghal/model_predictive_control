function Adot = Aqdot(theta,theta_dot,l)
A1 = zeros(3,6);
A2 = zeros(3,6);
A3 = zeros(3,6);
A4 = zeros(3,6);
A5 = zeros(3,6);
A6 = zeros(3,6);

A1(1,1) = l(1)*cos(theta(1)) + l(2)*cos(theta(1)+theta(2)) + l(3)*cos(theta(1)+theta(2)+theta(3));
A1(1,2) = l(2)*cos(theta(1)+theta(2)) + l(3)*cos(theta(1)+theta(2)+theta(3));
A1(1,3) = l(3)*cos(theta(1)+theta(2)+theta(3));
A1(2,1) = -(l(1)*sin(theta(1)) + l(2)*sin(theta(1)+theta(2)) + l(3)*sin(theta(1)+theta(2)+theta(3)));
A1(2,2) = -(l(2)*sin(theta(1)+theta(2)) + l(3)*sin(theta(1)+theta(2)+theta(3)));
A1(2,3) = -l(3)*sin(theta(1)+theta(2)+theta(3));

A2(1,1) = l(2)*cos(theta(1)+theta(2)) + l(3)*cos(theta(1)+theta(2)+theta(3));
A2(1,2) = l(2)*cos(theta(1)+theta(2)) + l(3)*cos(theta(1)+theta(2)+theta(3));
A2(1,3) = l(3)*cos(theta(1)+theta(2)+theta(3));
A2(2,1) = -(l(2)*sin(theta(1)+theta(2)) + l(3)*sin(theta(1)+theta(2)+theta(3)));
A2(2,2) = -(l(2)*sin(theta(1)+theta(2)) + l(3)*sin(theta(1)+theta(2)+theta(3)));
A2(2,3) = -l(3)*sin(theta(1)+theta(2)+theta(3));

A3(1,1) = l(3)*cos(theta(1)+theta(2)+theta(3));
A3(1,2) = l(3)*cos(theta(1)+theta(2)+theta(3));
A3(1,3) = l(3)*cos(theta(1)+theta(2)+theta(3));
A3(2,1) = -l(3)*sin(theta(1)+theta(2)+theta(3));
A3(2,2) = -l(3)*sin(theta(1)+theta(2)+theta(3));
A3(2,3) = -l(3)*sin(theta(1)+theta(2)+theta(3));

A4(1,4) = -(l(1)*cos(theta(4)) + l(2)*cos(theta(4)+theta(5)) + l(3)*cos(theta(4)+theta(5)+theta(6)));
A4(1,5) = -(l(2)*cos(theta(4)+theta(5)) + l(3)*cos(theta(4)+theta(5)+theta(6)));
A4(1,6) = -(l(3)*cos(theta(4)+theta(5)+theta(6)));
A4(2,4) = l(1)*sin(theta(4)) + l(2)*sin(theta(4)+theta(5)) + l(3)*sin(theta(4)+theta(5)+theta(6));
A4(2,5) = l(2)*sin(theta(4)+theta(5)) + l(3)*sin(theta(4)+theta(5)+theta(6));
A4(2,6) = l(3)*sin(theta(4)+theta(5)+theta(6));

A5(1,4) = -(l(2)*cos(theta(4)+theta(5)) + l(3)*cos(theta(4)+theta(5)+theta(6)));
A5(1,5) = -(l(2)*cos(theta(4)+theta(5)) + l(3)*cos(theta(4)+theta(5)+theta(6)));
A5(1,6) = -l(3)*cos(theta(4)+theta(5)+theta(6));
A5(2,4) = l(2)*sin(theta(4)+theta(5)) + l(3)*sin(theta(4)+theta(5)+theta(6));
A5(2,5) = l(2)*sin(theta(4)+theta(5)) + l(3)*sin(theta(4)+theta(5)+theta(6));
A5(2,6) = l(3)*sin(theta(4)+theta(5)+theta(6));

A6(1,4) = -l(3)*cos(theta(4)+theta(5)+theta(6));
A6(1,5) = -l(3)*cos(theta(4)+theta(5)+theta(6));
A6(1,6) = -l(3)*cos(theta(4)+theta(5)+theta(6));
A6(2,4) = l(3)*sin(theta(4)+theta(5)+theta(6));
A6(2,5) = l(3)*sin(theta(4)+theta(5)+theta(6));
A6(2,6) = l(3)*sin(theta(4)+theta(5)+theta(6));

Adot = A1*theta_dot(1) + A2*theta_dot(2) + A3*theta_dot(3) + A4*theta_dot(4) + A5*theta_dot(5) + A6*theta_dot(6);
end