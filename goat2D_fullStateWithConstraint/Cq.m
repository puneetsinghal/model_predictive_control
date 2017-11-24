function C = Cq(theta,theta_dot)
C = zeros(6,6);

C(1,1) = theta_dot(3)*(0.2*sin(theta(2)+theta(3))+0.1*sin(theta(3))) + ...
         theta_dot(2)*(0.2*sin(theta(2)+theta(3))+1.5*sin(theta(2)));
C(1,2) = theta_dot(3)*(0.2*sin(theta(2)+theta(3))+0.1*sin(theta(3))) + ...
         theta_dot(2)*(0.1*sin(theta(2)+theta(3))+0.75*sin(theta(2)));      
C(1,3) = theta_dot(3)*(0.1*sin(theta(2)+theta(3))+0.05*sin(theta(3)));

C(2,1) = theta_dot(1)*(-0.75*sin(theta(2))-0.1*sin(theta(2)+theta(3))) + ...
         theta_dot(3)*(0.1*sin(theta(3)));
C(2,2) = theta_dot(3)*(0.1*sin(theta(3)));
C(2,3) = theta_dot(3)*(0.05*sin(theta(3)));

C(3,1) = theta_dot(1)*(-0.1*sin(theta(2)+theta(3))-0.05*sin(theta(3))) + ...
         theta_dot(2)*(-0.1*sin(theta(3)));
C(3,2) = theta_dot(2)*(-0.05*sin(theta(3)));
% Symmetric
C(4,4) = theta_dot(6)*(0.2*sin(theta(5)+theta(6))+0.1*sin(theta(6))) + ...
         theta_dot(5)*(0.2*sin(theta(5)+theta(6))+1.5*sin(theta(5)));
C(4,5) = theta_dot(6)*(0.2*sin(theta(5)+theta(6))+0.1*sin(theta(6))) + ...
         theta_dot(5)*(0.1*sin(theta(5)+theta(6))+0.75*sin(theta(5)));      
C(4,6) = theta_dot(6)*(0.1*sin(theta(5)+theta(6))+0.05*sin(theta(6)));

C(5,4) = theta_dot(4)*(-0.75*sin(theta(5))-0.1*sin(theta(5)+theta(6))) + ...
         theta_dot(6)*(0.1*sin(theta(6)));
C(5,5) = theta_dot(6)*(0.1*sin(theta(6)));
C(5,6) = theta_dot(6)*(0.05*sin(theta(6)));

C(6,4) = theta_dot(4)*(-0.1*sin(theta(5)+theta(6))-0.05*sin(theta(6))) + ...
         theta_dot(5)*(-0.1*sin(theta(6)));
C(6,5) = theta_dot(5)*(-0.05*sin(theta(6)));

end