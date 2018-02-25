function gqOut = gq(theta,params)

theta11 = theta(1);
theta12 = theta(2);
theta13 = theta(3);

%%
mBody = params.m3;
mLink = params.m1;
lLink1 = params.l1;
lLink2 = params.l3;
rBody = params.l2;
%%
gqOut = [lLink1.*(0.98E1.*mBody+0.147E2.*mLink).*cos(theta11)+lLink2.*(0.98E1.* ...
  mBody+0.49E1.*mLink).*cos(theta11+theta12)+0.49E1.*mBody.*rBody.*cos( ...
  theta11+theta12+theta13),lLink2.*(0.98E1.*mBody+0.49E1.*mLink).*cos( ...
  theta11+theta12)+0.49E1.*mBody.*rBody.*cos(theta11+theta12+theta13), ...
  0.49E1.*mBody.*rBody.*cos(theta11+theta12+theta13)]';

end