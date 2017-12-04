function DgqOut = Dgq(theta, params)

theta11 = theta(1);
theta12 = theta(2);
theta13 = theta(3);

mBody = params.m3;
mLink = params.m1;
lLink1 = params.l1;
lLink2 = params.l3;
rBody = params.l2;

%%
DgqOut = [cos(theta11+theta12+theta13),(-1).*sin(theta11+theta12+theta13), ...
  lLink1.*cos(theta11)+lLink2.*cos(theta11+theta12)+(1/2).*rBody.*cos( ...
  theta11+theta12+theta13);sin(theta11+theta12+theta13),cos(theta11+ ...
  theta12+theta13),lLink1.*sin(theta11)+lLink2.*sin(theta11+theta12)+(1/2) ...
  .*rBody.*sin(theta11+theta12+theta13);0,0,1];

end

