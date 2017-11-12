function [ foot ] = joint2foot(teta)
%% You shoud load ForwardKinematics_US first! 
global leg
theta{1}=deg2rad(teta(1));
theta{2}=deg2rad(teta(2));
                                
%% Direct solving                                
% knees positions
leg.h1.k.p = forward_kinematics_hip_to_knee(leg, leg.h1,theta{1});
leg.h2.k.p = forward_kinematics_hip_to_knee(leg, leg.h2,theta{2});


% % I will use instead of Optimization, a numerical nonlinear equations solver
fun = @spheres_fwd;% nonlinear function representing the three spheres of possbile foot poisitions from each knee
x0 = [0,0,-200];
%options = optimoptions('fsolve','Display','iter');
foot= fsolve(fun,x0);%,options
foot = foot(:);

end

