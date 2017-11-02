%% calculates and stores the jacobian in a lookup table
load('ForwardKinematics_2D');
%uses the fwd_kmtcs cell array to calculate the jacobian
N = size(fwd_kmtcs,1)-1;%100
Jacobian = zeros(N*N,6);

dt = pi/100; % resolution of angles theta (discretization step)

for i = 1:N
    for j=1:N
          temp = zeros(3,2);
          temp(:,1) = (fwd_kmtcs{i+1,j} - fwd_kmtcs{i,j})/dt;
          temp(:,2) = (fwd_kmtcs{i,j+1} - fwd_kmtcs{i,j})/dt;
          temp = reshape(temp,1,6);
          Jacobian(N*(i-1)+j,:) = temp;
    end
end

save('Jacobian_lookup_table.mat', 'Jacobian');
