function L = LambdaAnalytical(theta,theta_dot,T,l)
M = Mq(theta);
C = Cq(theta,theta_dot);
G = Gq(theta);
A = Aq(theta,l);
Adot = Aqdot(theta,theta_dot,l);

L = (A*(M\A'))\(A*(M\(T-C*theta_dot-G))+Adot*theta_dot);
end