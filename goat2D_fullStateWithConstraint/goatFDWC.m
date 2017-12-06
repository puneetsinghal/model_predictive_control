function qdot = goatFDWC(t,q,u,l)
    [~,n] = size(u);
    if (n==1)
        qdot = goatFullDynamicsWithConstraints(q,u,l);
    end
    
end