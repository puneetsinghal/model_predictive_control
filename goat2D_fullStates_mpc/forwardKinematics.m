function pose = forwardKinematics(theta, lLink)

% g11 = transformation(0, 0, theta(1))*transformation(0, lLink(1)/2, 0);
% g12 = g11*transformation(0, lLink(1)/2, 0)*...
%     transformation(0, 0, theta(2))*transformation(0, lLink(2)/2, 0);
% g13 = g12*transformation(0, lLink(2)/2, 0)*...
%     transformation(0, 0, theta(3))*transformation(0, lLink(3)/2, 0);
% 
% g21 = transformation(0, 0, theta(4))*transformation(0, lLink(1)/2, 0);
% g22 = g21*transformation(0, lLink(1)/2, 0)*...
%     transformation(0, 0, theta(5))*transformation(0, lLink(2)/2, 0);
% g23 = g22*transformation(0, lLink(2)/2, 0)*...
%     transformation(0, 0, theta(6))*transformation(0, lLink(3)/2, 0);

% g11 = transformation(0, 0, theta(1))*transformation(0, lLink(1), 0);
% g12 = g11*transformation(0, 0, theta(2))*transformation(0, lLink(2), 0);
% g13 = g12*transformation(0, 0, theta(3))*transformation(0, lLink(3), 0);
% 
% g21 = transformation(0, 0, theta(4))*transformation(0, lLink(1), 0);
% g22 = g21*transformation(0, 0, theta(5))*transformation(0, lLink(2), 0);
% g23 = g22*transformation(0, 0, theta(6))*transformation(0, lLink(3), 0);

g11 = transformation(0, 0, theta(1))*transformation(lLink(1), 0, 0);
g12 = g11*transformation(0, 0, theta(2))*transformation(lLink(2), 0, 0);
g13 = g12*transformation(0, 0, theta(3))*transformation(lLink(3), 0, 0);

g21 = transformation(0, 0, theta(4))*transformation(lLink(1), 0, 0);
g22 = g21*transformation(0, 0, theta(5))*transformation(lLink(2), 0, 0);
g23 = g22*transformation(0, 0, theta(6))*transformation(lLink(3), 0, 0);

pose = [[0;0], g11(1:2,3), g12(1:2,3), g13(1:2,3), [0;0], g21(1:2,3), g22(1:2,3), g23(1:2,3)];

end

function g = transformation(x, y, theta)
g = [cos(theta), -sin(theta), x;...
    sin(theta), cos(theta), y;...
    0, 0, 1];
end