function dq = goatFullDynamicsCT(x, u, link_length)
    
    q = x(1:6);
    dq = x(7:12);
    M = Mq(q);
    Cor = CMatrix(q, dq);
    G = gq( q );
    
    B = [0 0;
         0 0;
         1 0;
         0 0;
         0 0;
         0 1];

    dq = [dq; M\( B*u - Cor * dq - G)];
    
end