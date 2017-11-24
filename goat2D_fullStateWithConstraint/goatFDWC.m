function qdot = goatFDWC(t,q,u,l)
    qdot = goatFullDynamicsWithConstraints(q,u,l);
end