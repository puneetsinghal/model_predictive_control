function dq = dynamicsCT(m, COM, I, y, z, L1, q, u)
    
    theta = q(1:4);
    dtheta = q(5:8);
    M = MMatrix_hebi(m, COM, I, y, z, L1, theta);
    
    C = CMatrix_hebi(m, COM, I, y, z, L1, theta, dtheta);
    
    G = GMatrix_hebi(m, COM, I, y, z, L1, theta);
    
    B = eye(4);
    
    M_cropped = M(1:4,1:4);
    C_cropped = C(1:4,1:4);
    G_cropped = G(1:4);
    
    dq = [dtheta; pinv(M_cropped)*( B*u - C_cropped * dtheta - G_cropped)];
end