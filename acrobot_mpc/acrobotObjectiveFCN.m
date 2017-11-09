%Cost Function
function J = acrobotObjectiveFCN(u, xk, xref, u0, params)

    Q = diag([100; 100; 0.01; 0.01]); %[10000*eye(4), zeros(4);zeros(4), 0.01*eye(4)];
    R = 1*eye(1);

    J = 0;
    xk1 = xk;
    N = params.N;

    for i = 1:N
        uk = u(:,i);
        xk1 = acrobotDynamicsDT(xk1, uk, params);
        
%         J = J + (xk1-xref)'*Q*(xk1-xref);
        J = J + (xk1)'*Q*(xk1);

        if i ==1
            J = J + (uk-u0)' * R * (uk-u0);
        else
            J = J + (uk-u(i-1))' * R * (uk-u(i-1));
        end
    end     
end