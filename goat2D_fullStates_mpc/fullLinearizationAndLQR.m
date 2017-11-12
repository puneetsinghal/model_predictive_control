function [B, K] = fullLinearizationAndLQR(q0, dq0, uStar)
    M = Mq(q0);
    Cor = CMatrix( q0, dq0);
    B = [0 0;
         0 0;
         1 0;
         0 0;
         0 0;
         0 1];

    G = gq(q0);
    DGHat = Dgq(q0);
    DB1Hat = DB1q(q0);
    DB2Hat = DB2q(q0);

    Alin = [ zeros(6), eye(6);...
        (-M\(DGHat + uStar(1)*DB1Hat + uStar(2)*DB2Hat)), (-M\Cor)];
    Blin = [zeros(6,2); M\B];
    rank(ctrb(Alin,Blin))

    Q = diag([10000, 10000, 10000, 10000, 10000, 10000, 1, 1, 1, 1, 1, 1]);
    R = diag([0.01, 0.01]);
    K = -1*lqr(Alin, Blin, Q, R);
end