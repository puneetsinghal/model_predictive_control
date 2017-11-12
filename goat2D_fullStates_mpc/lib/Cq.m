function CqOut = Cq(theta,Dtheta)

theta11 = theta(1);
theta12 = theta(2);
theta13 = theta(3);
theta21 = theta(4);
theta22 = theta(5);
theta23 = theta(6);

Dtheta11 = Dtheta(1);
Dtheta12 = Dtheta(2);
Dtheta13 = Dtheta(3);
Dtheta21 = Dtheta(4);
Dtheta22 = Dtheta(5);
Dtheta23 = Dtheta(6);
%%
% theta11 = 0;
% theta12 = 0;
% theta13 = 0;
% theta21 = 0;
% theta22 = 0;
% theta23 = 0;
% 
% Dtheta11 = 0;
% Dtheta12 = 0;
% Dtheta13 = 0;
% Dtheta21 = 0;
% Dtheta22 = 0;
% Dtheta23 = 0;

%%
CqOut = [(1/2).*(Dtheta11.*(Dtheta12.*((-0.401E2).*sin(theta12)+(-0.8E1).*sin( ...
  theta12+theta13))+Dtheta13.*((-0.4E1).*sin(theta13)+(-0.8E1).*sin( ...
  theta12+theta13)))+Dtheta12.*Dtheta13.*((-0.4E1).*sin(theta13)+(-0.8E1) ...
  .*sin(theta12+theta13))+Dtheta13.^2.*((-0.2E1).*sin(theta13)+(-0.4E1).* ...
  sin(theta12+theta13))+Dtheta12.^2.*((-0.2005E2).*sin(theta12)+(-0.4E1).* ...
  sin(theta12+theta13))),(-0.2E1).*Dtheta11.*Dtheta13.*sin(theta13)+(( ...
  -0.2E1).*Dtheta12+(-1.E0).*Dtheta13).*Dtheta13.*sin(theta13)+ ...
  Dtheta11.^2.*(0.10025E2.*sin(theta12)+0.2E1.*sin(theta12+theta13)), ...
  0.2E1.*Dtheta11.*Dtheta12.*sin(theta13)+0.1E1.*Dtheta12.^2.*sin(theta13)+...
  Dtheta11.^2.*(0.1E1.*sin(theta13)+0.2E1.*sin(theta12+theta13)),(1/2).*( ...
  Dtheta21.*(Dtheta22.*((-0.401E2).*sin(theta22)+(-0.8E1).*sin(theta22+ ...
  theta23))+Dtheta23.*((-0.4E1).*sin(theta23)+(-0.8E1).*sin(theta22+ ...
  theta23)))+Dtheta22.*Dtheta23.*((-0.4E1).*sin(theta23)+(-0.8E1).*sin( ...
  theta22+theta23))+Dtheta23.^2.*((-0.2E1).*sin(theta23)+(-0.4E1).*sin( ...
  theta22+theta23))+Dtheta22.^2.*((-0.2005E2).*sin(theta22)+(-0.4E1).*sin( ...
  theta22+theta23))),(-0.2E1).*Dtheta21.*Dtheta23.*sin(theta23)+((-0.2E1) ...
  .*Dtheta22+(-1.E0).*Dtheta23).*Dtheta23.*sin(theta23)+Dtheta21.^2.*( ...
  0.10025E2.*sin(theta22)+0.2E1.*sin(theta22+theta23)),0.2E1.*Dtheta21.* ...
  Dtheta22.*sin(theta23)+0.1E1.*Dtheta22.^2.*sin(theta23)+Dtheta21.^2.*( ...
  1.E0.*sin(theta23)+0.2E1.*sin(theta22+theta23))];

end