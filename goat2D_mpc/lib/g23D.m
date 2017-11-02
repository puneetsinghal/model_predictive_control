function g23Out = g23D(theta)

theta11 = theta(1);
theta12 = theta(2);
theta13 = theta(3);
theta21 = theta(4);
theta22 = theta(5);
theta23 = theta(6);
%%
% theta11 = 0;
% theta12 = 0;
% theta13 = 0;
% theta21 = 0;
% theta22 = 0;
% theta23 = 0;

%%
g23Out = [cos(0.15708E1+(-1).*theta21+(-1).*theta22+(-1).*theta23),sin(0.15708E1+ ...
  (-1).*theta21+(-1).*theta22+(-1).*theta23),(-1).*sin(0.56936E0+theta21)+ ...
  0.5E0.*sin(0.74529E0+(-1).*theta21+(-1).*theta22)+0.2E0.*sin(0.15708E1+( ...
  -1).*theta21+(-1).*theta22+(-1).*theta23);(-1).*sin(0.15708E1+(-1).* ...
  theta21+(-1).*theta22+(-1).*theta23),cos(0.15708E1+(-1).*theta21+(-1).* ...
  theta22+(-1).*theta23),cos(0.56936E0+theta21)+0.5E0.*cos(0.74529E0+(-1) ...
  .*theta21+(-1).*theta22)+0.2E0.*cos(0.15708E1+(-1).*theta21+(-1).* ...
  theta22+(-1).*theta23);0,0,0.1E1];
