function [dq, q1] = dynamics(t, q, u_initial, q10, xNominal, K, B, drawSwitch)
% input 1: t: time of simulation, start and end time are input arguments in
% ODE
    global global_link_length
    global hlink hjnt
    q2 = q(1:3);
    dq2 = q(4:6);
%     q1 = fsolve(@(q1) constraints(q1,q2), q10);
%     q1 = findFeasibleConfiguration(q2, q10, global_link_length);
    q1 = findFeasibleConfigurationAnalytical(q2, global_link_length);
%     q1(2:3) = wrapToPi(q1(2:3));
%     q1 = wrapToPi(q1);
    M = Mq([q1;q2]);
    H = Hq([q1;q2]);
    M_hat = [H, eye(3)] * M * [H;eye(3)];
    dq1 = H*dq2;
    
    HDot = HDotq([q1; q2] , [dq1; dq2]);
    
    C0 = CMatrix( [q1; q2] , [dq1; dq2] );
    C_hat = [H, eye(3)] * M * [HDot; zeros(3)] + [H, eye(3)] * C0 * [H; eye(3)];
    
    gR = gq( [ q1 ; q2] );
    G_hat = [H, eye(3)] * gR;
    
%     B = [0 0;0 0;1 0;0 0;0 0;0 1];
    u = K*(q - xNominal) + u_initial;
    dq = [dq2; M_hat\( [H, eye(3)]*B*u - C_hat * dq2 - G_hat)];
%     q10 = q1
%     display(t);
    if(drawSwitch)
        pose = forwardKinematics([q1; q2], global_link_length);
        for i =1:length(global_link_length)
            set(hlink(i),...
                'XData', [pose(1,i); pose(1,i+1)],...
                'YData', [pose(2,i); pose(2,i+1)]);
            set(hjnt(i), 'XData', pose(1,i),'YData', pose(2,i));
        end
        for i =4:2*length(global_link_length)
            set(hlink(i),...
                'XData', [pose(1,i+1); pose(1,i+2)],...
                'YData', [pose(2,i+1); pose(2,i+2)]);
            set(hjnt(i), 'XData', pose(1,i+2),'YData', pose(2,i+2));
        end 
        drawnow
    end
    
end
    