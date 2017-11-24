function G = Gq(theta)
G = zeros(6,1);
G(1) = -24.5*cos(theta(1))-7.35*cos(theta(1)+theta(2))-0.98*cos(theta(1)+theta(2)+theta(3));
G(2) = -7.35*cos(theta(1)+theta(2))-0.98*cos(theta(1)+theta(2)+theta(3));
G(3) = -0.98*cos(theta(1)+theta(2)+theta(3));
G(4) = -24.5*cos(theta(4))-7.35*cos(theta(4)+theta(5))-0.98*cos(theta(4)+theta(5)+theta(6));
G(5) = -7.35*cos(theta(4)+theta(5))-0.98*cos(theta(4)+theta(5)+theta(6));
G(6) = -0.98*cos(theta(4)+theta(5)+theta(6));
end