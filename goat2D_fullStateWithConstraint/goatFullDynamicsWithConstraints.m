function qdot = goatFullDynamicsWithConstraints(t,q,u,l)
    theta = q(1:6,1);
    theta_dot = q(7:12,1);
    qdot = zeros(12,1);
    qdot(1:6,1) = q(7:12,1);
    M = Mq(theta);
    C = Cq(theta,theta_dot);
    G = Gq(theta);
    A = Aq(theta,l);
    L = LambdaAnalytical(theta,theta_dot,u,l);
    qdot(7:12) = M\(u - A'*L - G - C*theta_dot);    
end