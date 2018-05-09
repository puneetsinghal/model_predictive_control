function dq =acrobotDynamicsCT(t,q, u, params)
    %%
		%params.m1
    m1 = params.m1;
    m2 = params.m2;
    l1 = params.l1;
    l2 = params.l2;
    lc1 = l1/2;
    lc2 = l2/2; 
    g = params.g;
    I1 = params.I1;
    I2 = params.I2;
    theta_1 = q(1);
    theta_2 = q(2);
    dtheta_1 = q(3);
    dtheta_2 = q(4);
    
    %%
    M = zeros(2,2);
%     M(1,1) = m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(theta_2)) + I1 + I2;
%     M(1,2) = m2*(lc2^2 + l1*lc2*cos(theta_2)) + I2;
%     M(2,1) = M(1,2);
%     M(2,2) = m2*lc2^2 + I2;

    M(1,1) = m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(theta_2)) + I1 + I2;
    M(1,2) = m2*(lc2^2 + l1*lc2*cos(theta_2)) + I2;
    M(2,1) = M(1,2);
    M(2,2) = m2*lc2^2 + I2;

    C = [-2*m2* l1*lc2*sin(theta_2)*dtheta_2, -m2*l1*lc2*sin(theta_2)* dtheta_2;...
        m2*l1*lc2*sin(theta_2)*dtheta_1, 0];

    G = [(m1*lc1 + m2*l1)*g*cos(pi/2+theta_1) + m2* lc2*g*cos(pi/2+theta_1 + theta_2);...
        m2*lc2*g*cos(pi/2+theta_1 + theta_2)];
    
    B = [0; 1];
    %%
    dq = [dtheta_1; dtheta_2; pinv(M)*( B*u - C* [dtheta_1; dtheta_2] - G)];
end
