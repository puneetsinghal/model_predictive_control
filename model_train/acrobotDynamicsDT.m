function [xk1, yk] = acrobotDynamicsDT(xk, uk, params)
    % Repeat application of Euler method sampled at Ts/M.
    M =  10;
    delta = (params.Ts)/M;
    xk1 = xk;
    for ct=1:M
        xk_dot = acrobotDynamicsCT(xk1, uk, params);
        xk1 = xk1 + delta*xk_dot;
    end
    yk = xk;
end