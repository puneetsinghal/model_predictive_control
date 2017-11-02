function [ joints ] = get_joints( foot, inv_kmtcs)
% Does inverse kinematics i.e. given a foot position, returns the corresponding joint angles 
% input: 
    % - foot: foot position, a 3 - dimensional vector (r, theta, phi)
% output:
    % - joints: 3 joint angles of hip actuators
    
        N = size(inv_kmtcs,1)-1;
        r = foot(1);
        theta = deg2rad(foot(2));
        phi = deg2rad(foot(3));
        
        dr = (18-7.5)/N;
        dtheta = 2*pi/N;
        dphi = pi/(2*N);
        
        index1 = uint8(floor((r-7.5)*N/(18-7.5)+1)); %index of the first joint in the cell
        index2 = uint8(floor((theta)*N/(2*pi)+1)); %index of the second joint in the cell array
        index3 = uint8(floor((phi)*2*N/pi+1)); %index of the third joint in the cell array
               
        joints = inv_kmtcs{index1, index2, index3} + (inv_kmtcs{min(index1 + 1, N+1), index2, index3} - inv_kmtcs{index1, index2, index3})/dr*mod(r - 7.5,dr)...
                                                   + (inv_kmtcs{index1, min(index2 + 1, N+1), index3} - inv_kmtcs{index1, index2, index3})/dtheta*mod(theta, dtheta)...
                                                   + (inv_kmtcs{index1, index2, min(index3 + 1, N+1)} - inv_kmtcs{index1, index2, index3})/dphi*mod(phi, dphi);

end

