function [J] = get_jacobian (theta, Jacobian)
% returns the jacobian given a 3 dimentional vector theta representing the hip angles (in degress)
% input:
    % - theta: hip angles in degrees
    % - Jacobian: 100x100x100 lookup table, appears when you load('Jacobian_lookup_table_US.mat')
% output:
    % J: Jacobian at the corresponding configuration

% NOTE: linear interpolation of the values in the lookup table is used

theta = deg2rad(theta);
N = size(Jacobian,1); % resolution of table(100)

ind1 = uint8(floor((theta(1) + pi/2)*N/pi+1));
ind2 = uint8(floor((theta(2) + pi/2)*N/pi+1));
ind3 = uint8(floor((theta(3) + pi/2)*N/pi+1));
dt = pi/N; % resolution of angles

J = Jacobian{ind1, ind2, ind3} + (Jacobian{min(ind1 + 1, N), ind2, ind3} - Jacobian{ind1, ind2, ind3})/dt * ( mod(theta(1)+pi/2,dt) )...
                               + (Jacobian{ind1, min(ind2 + 1, N), ind3} - Jacobian{ind1, ind2, ind3})/dt * ( mod(theta(2)+pi/2,dt) )...
                               + (Jacobian{ind1, ind2, min(ind3 + 1, N)} - Jacobian{ind1, ind2, ind3})/dt * ( mod(theta(3)+pi/2,dt) );

end
