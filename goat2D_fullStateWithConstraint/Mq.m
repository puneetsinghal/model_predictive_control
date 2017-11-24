function M = Mq(theta)
M = zeros(6,6);
M(1,1) = -0.2*cos(theta(2)+theta(3))-0.1*cos(theta(3))-1.5*cos(theta(2))-2.68;
M(1,2) = -0.1*cos(theta(2)+theta(3))-0.1*cos(theta(3))-0.75*cos(theta(2))-0.3466;
M(1,3) = -0.1*cos(theta(2)+theta(3))-0.05*cos(theta(3))-0.0133;

M(2,1) = -0.1*cos(theta(2)+theta(3))-0.1*cos(theta(3))-0.75*cos(theta(2))-0.3466;
M(2,2) = -0.1*cos(theta(3))-0.3466;
M(2,3) = -0.05*cos(theta(3))-0.0133;

M(3,1) = -0.1*cos(theta(2)+theta(3))-0.05*cos(theta(3))-0.0133;
M(3,2) = -0.05*cos(theta(3))-0.0133;
M(3,3) = -0.0133;

% Symmetric 

M(4,4) = -0.2*cos(theta(5)+theta(6))-0.1*cos(theta(6))-1.5*cos(theta(5))-2.68;
M(4,5) = -0.1*cos(theta(5)+theta(6))-0.1*cos(theta(6))-0.75*cos(theta(5))-0.3466;
M(4,6) = -0.1*cos(theta(5)+theta(6))-0.05*cos(theta(6))-0.0133;

M(5,4) = -0.1*cos(theta(5)+theta(6))-0.1*cos(theta(6))-0.75*cos(theta(5))-0.3466;
M(5,5) = -0.1*cos(theta(6))-0.3466;
M(5,6) = -0.05*cos(theta(6))-0.0133;

M(6,4) = -0.1*cos(theta(5)+theta(6))-0.05*cos(theta(6))-0.0133;
M(6,5) = -0.05*cos(theta(6))-0.0133;
M(6,6) = -0.0133;

end