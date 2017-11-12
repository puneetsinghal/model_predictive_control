function dx = fullDynamics(t, x, u_initial, xNominal, K, B, drawSwitch)
% input 1: t: time of simulation, start and end time are input arguments in
% ODE
    global global_link_length
    global hlink hjnt
    q = x(1:6);
    dq = x(7:12);
    M = Mq(q);
    Cor = CMatrix( q , dq );
    G = gq(q);
    
%     B = [0 0;
%          0 0;
%          1 0;
%          0 0;
%          0 0;
%          0 1];
    
     u = K*(x - xNominal) + u_initial;
    
    dx = [dq; M\( B*u - Cor * dq - G)];

    %%
    q1 = q(1:3);
    q2 = q(4:6);
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
    