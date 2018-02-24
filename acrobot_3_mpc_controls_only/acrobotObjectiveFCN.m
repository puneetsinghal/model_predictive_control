%Cost Function
function J = acrobotObjectiveFCN(p, x0, xref, u0, params)

    Q = diag([1000; 1000; 1000; 1; 1; 1]); %[10000*eye(4), zeros(4);zeros(4), 0.01*eye(4)];
    R = diag(1);
    J = 0;
    N = params.N;
   
    xk = x0;
    for tk = 1:N
        uk = p(tk);
        
        xk = acrobotDynamicsDT(xk, uk, params);
        
        J = J + (xk-xref)'*Q*(xk-xref);
        if tk==1
            J = J + (uk-u0)' * R * (uk-u0);
        else
            J = J + (uk-p(tk-1))' * R * (uk-p(tk-1));
        end
    end
end