function [B, K] = linearizationAndLQR(q10, q20, dq20, uStar)
    M = Mq([q10;q20]);
    H = Hq([q10;q20]);
    M_hat = [H, eye(3)] * M * [H;eye(3)];

    dq10 = H*dq20;
    HDot = HDotq([q10; q20] , [dq10; dq20]);

    C0 = CMatrix( [q10; q20] , [dq10; dq20] );
    C_hat = [H, eye(3)] * M * [HDot; zeros(3)] + [H, eye(3)] * C0 * [H; eye(3)];

    B = [0 0;
         0 0;
         1 0;
         0 0;
         0 0;
         0 1];

    B_hat = [H, eye(3)]*B;

    gR = gq( [ q10 ; q20] );
    G_hat = [H, eye(3)] * gR;
    DGHat = Dgq([q10; q20] );
    DB1Hat = DB1q( [q10; q20] );
    DB2Hat = DB2q( [q10; q20] );

    Alin = [ zeros(3), eye(3);...
        (-M_hat\(DGHat + uStar(1)*DB1Hat + uStar(2)*DB2Hat)), (-M_hat\C_hat)];
    Blin = [zeros(3,2); M_hat\B_hat];
    rank(ctrb(Alin,Blin))

    Q = diag([10000, 10000, 10000, 1, 1, 1]);
    R = diag([0.01, 0.01]);
    K = -1*lqr(Alin, Blin, Q, R);
end