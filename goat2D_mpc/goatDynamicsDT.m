function [xk1, yk] = goatDynamicsDT(xk, uk, Ts, link_length)
    % Repeat application of Euler method sampled at Ts/M.
    M = 10;
    delta = Ts/M;
    xk1 = xk;
    for ct=1:M
        ct;
        xk_dot = goatDynamicsCT(xk1,uk, link_length);
        xk1(1:6) = xk1(1:6) + delta*xk_dot(1:6);
        xk1(7:9) = xk_dot(7:9);
    end
    yk = xk;
end