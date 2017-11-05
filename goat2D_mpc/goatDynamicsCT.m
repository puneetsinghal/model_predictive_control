function dq =goatDynamicsCT(q, u, link_length)
    q10 = q(7:9);
    q = q(1:6);
    q2 = q(1:3);
    dq2 = q(4:6);
%   q1 = fsolve(@(q1) constraints(q1,q2), q10);
    q1 = findFeasibleConfiguration(q2, q10, link_length);
    
%     q1(2:3) = wrapToPi(q1(2:3));
    q1 = wrapToPi(q1);
    M = Mq([q1;q2]);
    H = Hq([q1;q2]);
    M_hat = [H, eye(3)] * M * [H;eye(3)];
    dq1 = H*dq2;
    
    HDot = HDotq([q1; q2] , [dq1; dq2]);
    
    C0 = CMatrix( [q1; q2] , [dq1; dq2] );
    C_hat = [H, eye(3)] * M * [HDot; zeros(3)] + [H, eye(3)] * C0 * [H; eye(3)];
    
    gR = gq( [ q1 ; q2] );
    G_hat = [H, eye(3)] * gR;
    
    B = [0 0;
         0 0;
         1 0;
         0 0;
         0 0;
         0 1];

    B_hat = [H, eye(3)]*B;

    dq = [dq2; M_hat\( B_hat*u - C_hat * dq2 - G_hat); q1];
    
end