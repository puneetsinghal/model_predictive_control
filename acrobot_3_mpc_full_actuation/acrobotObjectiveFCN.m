%Cost Function
function J = acrobotObjectiveFCN(p, xref, u0, params)

    Q = diag([1000; 1000; 1000; 0.01; 0.01; 0.01]); %[10000*eye(4), zeros(4);zeros(4), 0.01*eye(4)];
    R = diag([0.001; 0.001; 0.001]);
%     Q = [100; 100; 0.01; 0.01];
%     p = p- [repmat(xref,[1,params.N]);[u0, p(5,1:end-1)]];
%     p = reshape(p,[],1);
%     J = sum(p'*diag(repmat([Q;R],[params.N,1]))*p);
    J = 0;
%     u = p(7,:);
    N = params.N;
    
%     for i = 1:N-1
%         uk = u(:,i);
%         xk1 = p(1:6,i);
% %         xk1 = acrobotDynamicsDT(xk1, uk, params);
%         
% %         J = J + (xk1-xref)'*Q*(xk1-xref);
%         J = J + (xk1)'*Q*(xk1);
% 
%         if i ==1
%             J = J + (uk-u0)' * R * (uk-u0);
%         else
%             J = J + (uk-u(i-1))' * R * (uk-u(i-1));
%         end
%     end    
    xN = p(1:6,N);
    J = J + ((xN-xref)'*Q*(xN-xref));
    for tk = 1:N-1
        xk = p(1:6,tk);
        uk = p(7:end,tk);
        
        xk1 = p(1:6,tk+1);
        uk1 = p(7:end,tk+1);
        
        J = J + (xk)'*Q*(xk) + (xk1)'*Q*(xk1);
        
        J = J + (uk)' * R * (uk) + (uk1)' * R * (uk1);
    end
end