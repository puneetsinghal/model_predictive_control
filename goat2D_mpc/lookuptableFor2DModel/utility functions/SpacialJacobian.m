function [ Js ] = SpacialJacobian( N,T,teta )
%returns the spacial jacobian that relates the N joints' velocities to the Nth frame spacial velocity
Js=zeros(6,N);

Js(1:6,1)=T(1:6,1);

    for i=2:N
        Tprime=eye(4);%  twist accounting for the movement of joints (transformed twist)
        for j=1:i-1
            Tprime=Tprime*expm(teta(j)*wedge(T(1:6,j)));
        end
        Js(1:6,i)=Adjoint(Tprime)*T(1:6,i);
    end
end
