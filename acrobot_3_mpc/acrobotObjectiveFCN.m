%Cost Function
function J = acrobotObjectiveFCN(p, xref, params)
%     Q = [100; 100; 0.01; 0.01];
%     p = p- [repmat(xref,[1,params.N]);[u0, p(5,1:end-1)]];
%     p = reshape(p,[],1);
%     J = sum(p'*diag(repmat([Q;R],[params.N,1]))*p);
    N = params.N;
    Q = ones(6,1);
    Q(1:6,1) = 10*Q(1:6,1);
    R = 0.01*ones(1,1);
    Q = Q*ones(1,N);
    R = R*ones(1,N);
    xref = [xref*ones(1,N);zeros(1,N)];
    J = (p-xref).*[Q;R].*(p-xref);
    J=sum(sum(J));    
end