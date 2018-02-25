function dq =acrobotDynamicsCT(x, u, params)
    %%
    m1 = params.m1;
    m2 = params.m2;
    l1 = params.l1;
    l2 = params.l2;
    lc1 = l1/2;
    lc2 = l2/2; 
    g = params.g;
    I1 = params.I1;
    I2 = params.I2;
    
    %%
    q = x(1:3);
    dq = x(4:6);
    
    M = Mq(q, params);
    Cor = CMatrix(q, dq, params);
    G = gq( q, params );
    
    B = [0;
         0;
         1];
%     B = eye(3);

    dq = [dq; M\( B*u - Cor * dq - G)];
end