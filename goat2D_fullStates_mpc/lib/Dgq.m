function DgqOut = Dgq(theta)

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
% 
%%
DgqOut = [(-0.19747E3).*sin(theta11)+(-0.98245E2).*sin(theta11+theta12)+( ...
  -0.196E2).*sin(theta11+theta12+theta13),(-0.98245E2).*sin(theta11+ ...
  theta12)+(-0.196E2).*sin(theta11+theta12+theta13),(-0.196E2).*sin( ...
  theta11+theta12+theta13),0,0,0;(-0.98245E2).*sin(theta11+theta12)+( ...
  -0.196E2).*sin(theta11+theta12+theta13),(-0.98245E2).*sin(theta11+ ...
  theta12)+(-0.196E2).*sin(theta11+theta12+theta13),(-0.196E2).*sin( ...
  theta11+theta12+theta13),0,0,0;(-0.196E2).*sin(theta11+theta12+theta13), ...
  (-0.196E2).*sin(theta11+theta12+theta13),(-0.196E2).*sin(theta11+ ...
  theta12+theta13),0,0,0;0,0,0,(-0.19747E3).*sin(theta21)+(-0.98245E2).* ...
  sin(theta21+theta22)+(-0.196E2).*sin(theta21+theta22+theta23),( ...
  -0.98245E2).*sin(theta21+theta22)+(-0.196E2).*sin(theta21+theta22+ ...
  theta23),(-0.196E2).*sin(theta21+theta22+theta23);0,0,0,(-0.98245E2).* ...
  sin(theta21+theta22)+(-0.196E2).*sin(theta21+theta22+theta23),( ...
  -0.98245E2).*sin(theta21+theta22)+(-0.196E2).*sin(theta21+theta22+ ...
  theta23),(-0.196E2).*sin(theta21+theta22+theta23);0,0,0,(-0.196E2).*sin( ...
  theta21+theta22+theta23),(-0.196E2).*sin(theta21+theta22+theta23),( ...
  -0.196E2).*sin(theta21+theta22+theta23)];

end

