function [xk1, yk] = dynamicsDT(m, COM, I, y, z, L1, xk, uk, Ts)
    % Repeat application of Euler method sampled at Ts/M.
    M =  10;
    delta = Ts/M;
    xk1 = xk;
    for ct=1:M
        xk_dot = dynamicsCT(m, COM, I, y, z, L1, xk1, uk);
        xk1 = xk1 + delta*xk_dot;
    end
    yk = xk;
end