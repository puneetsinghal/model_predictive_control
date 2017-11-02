function [ P ] = get_foot_position( teta,fwd_kmtcs )
%Return the foot position corresponding to an input joint angle vector
% input:
    % - theta: 3-dimensional vector containing the three hip joint angles
    % - fwd_kmtcs: forward kinematics lookup table
% output:
    % - P: [x,y,z] cartesion position of the foot
    
theta{1}=deg2rad(teta(1));
theta{2}=deg2rad(teta(2));
theta{3}=deg2rad(teta(3));

%Using Look-up table
N = size(fwd_kmtcs,1)-1;
ind1 = uint8(floor((theta{1} + pi/2)*N/pi+1));
ind2 = uint8(floor((theta{2} + pi/2)*N/pi+1));
ind3 = uint8(floor((theta{3} + pi/2)*N/pi+1));
dt = pi/N;
P = fwd_kmtcs{ind1,ind2,ind3} + (fwd_kmtcs{min(ind1+1,N), ind2, ind3} - fwd_kmtcs{ind1, ind2, ind3})/dt * ( mod(theta{1}+pi/2,dt) )...
                                    + (fwd_kmtcs{ind1, min(ind2 + 1,N), ind3} - fwd_kmtcs{ind1, ind2, ind3})/dt * ( mod(theta{2}+pi/2,dt) )...
                                    + (fwd_kmtcs{ind1, ind2 , min(ind3+1,N)} - fwd_kmtcs{ind1, ind2, ind3})/dt * ( mod(theta{3}+pi/2,dt) );
                                
end

