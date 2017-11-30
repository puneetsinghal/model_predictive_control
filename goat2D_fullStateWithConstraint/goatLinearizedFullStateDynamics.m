function [A,B] = goatLinearizedFullStateDynamics(q,u,l)
%% Approximate method
% Perturb each state by epsilon and observe change
epsilon = 1e-7;
A = zeros(length(q));
B = zeros(length(q),2);
for i=1:1:length(q)
    u_ = zeros(6,1);
    u_(3) = u(1);
    u_(6) = u(2);
    q_ = q;
    q__ = q;
    q_(i) = q(i)-epsilon;
    q__(i) = q(i)+epsilon;
    qdot_ = goatFullDynamicsWithConstraints(q_,u_,l);
    qdot__ = goatFullDynamicsWithConstraints(q__,u_,l);
    A(:,i) = (qdot__ - qdot_)/(2*epsilon);
end

for i=3:3:6
    u_ = zeros(6,1);
    u__ = zeros(6,1);
    u_(i) = u(i/3)-epsilon;
    u__(i) = u(i/3)+epsilon;
    qdot_ = goatFullDynamicsWithConstraints(q,u_,l);
    qdot__ = goatFullDynamicsWithConstraints(q,u__,l);
    B(:,i/3) = (qdot__ - qdot_)/(2*epsilon);
end




end