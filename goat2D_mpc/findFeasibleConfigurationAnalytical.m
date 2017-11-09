function theta_1 = findFeasibleConfigurationAnalytical(theta_2, link_lengths)
    theta_1 = zeros(3,1);
    k = sum(theta_2);
    l = link_lengths;
    Sx = l(1)*cos(theta_2(1)) + l(2)*cos(theta_2(1) + theta_2(2))+ l(3)*cos(k);
    Sy = l(1)*sin(theta_2(1)) + l(2)*sin(theta_2(1) + theta_2(2))+ l(3)*sin(k);
    X = Sx - l(3)*cos(k - pi);
    Y = Sy - l(3)*sin(k - pi);
    costheta5 = (l(1)^2 + l(2)^2 - X^2 - Y^2)/(2*l(1)*l(2));
    theta_1(2) = acos(costheta5) - pi; % Must be less than pi/2
    delta = atan2(Y,X);
    if (delta < 0)
        fprintf('Check delta!!%f\n',delta)
    end
    theta_1(1) = asin((l(2)*sin(pi+theta_1(2)))/sqrt(X^2 + Y^2)) + delta;
    X1 = l(1)*cos(theta_1(1));
    Y1 = l(1)*sin(theta_1(1));
    theta_1(3) = acos((l(2)^2 + l(3)^2 -(Sx-X1)^2 - (Sy-Y1)^2)/(2*l(2)*l(3))) - pi;
end