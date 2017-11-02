function otherLegJoints = findFeasibleConfiguration(theta2, theta1_0, link_length)
% given the joint angles of left/right leg returns the joint angles of the 
% other right/left leg for a feasible configuration 
%  input 1: - theta2: joint angles of one of the legs
%  input 2: - theta1_initial: joint angles of other legs: starting point for optimization
% input 3: link_lengths: vector of 3 link lengths [lLink1, lLink2, rBody]
%  output: - otherLegJoints: joint angles of the other leg

% theta2 = [deg2rad(60),deg2rad(60),deg2rad(60)]';

params.theta2 = theta2;
params.linkLength = link_length;
options = optimset('Display','None','MaxFunEvals',1000000,'Algorithm','sqp');

lb = [ -pi, -pi, -pi/2 ]';
ub = [ pi, pi, pi/2 ]';

Constraints = @(theta1)constraints(theta1, params);
[otherLegJoints,~,~]=fmincon(@(theta1)criterion(theta1,params),theta1_0,[],[],[],[],lb,ub,Constraints,options);

end

function objective = criterion(theta1, params)
%%% optimization criterion: p is vector of joint angles

pose = forwardKinematics([theta1; params.theta2], params.linkLength);
% format of pose = [[0;0], g11(1:2,3), g12(1:2,3), g13(1:2,3), [0;0], g21(1:2,3), g22(1:2,3), g23(1:2,3)];

Ceq(1,1) = pose(1 , 4) - pose(1 , 8);
Ceq(2,1) = pose(2 , 4) - pose(2 , 8);
Ceq(3,1) = pi - abs((sum(theta1) - sum(params.theta2)));
Q = [5, 0, 0;...
    0, 5, 0;...
    0, 0, 1];
objective = Ceq'*Q*Ceq;
% objective=1;

end

function [C, Ceq] = constraints(theta1 , params)

pose = forwardKinematics([theta1; params.theta2], params.linkLength);
% format of pose = [[0;0], g11(1:2,3), g12(1:2,3), g13(1:2,3), [0;0], g21(1:2,3), g22(1:2,3), g23(1:2,3)];

% equality constraints
Ceq(1) = pose(1 , 4) - pose(1 , 8);
Ceq(2) = pose(2 , 4) - pose(2 , 8);
Ceq(3) = pi - abs((sum(theta1) - sum(params.theta2)));

% inequality constraints
C=[];

end
