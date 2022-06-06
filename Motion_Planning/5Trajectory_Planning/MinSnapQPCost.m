%{
Function to calculate the H matrix and f vector for the cost of QP
%}


function [H, f] = MinSnapQPCost(ts, n_order, n_poly)

	% compute H,  H is a Symmetric matrix
    % H_1 =   [0    0   0]
    %         [0    f   f]
    %         [0    f   f]          % f =  integral of ts(i) ~ ts(i+1) 
    % H_x =   [H_1  0     0    0]
    %         [0    H_2   0    0]
    %         [0    0   H_...  0]
    %         [0    0     0  H_n]   % n = n_poly
    % H_all = [H_x  0     0]
    %         [0    H_y   0]
    %         [0    0   H_z]        % H_x == H_y == H_z

    % H and f
    H_x = [];

	for i = 1:n_poly
	    H_x = blkdiag(H_x, calc_Q(n_order, 4, ts(i), ts(i+1)));  % for a single dimension
    end

    zeroMat = zeros(size(H_x));
    H = [H_x, zeroMat, zeroMat; zeroMat, H_x, zeroMat; zeroMat, zeroMat, H_x];  % quadratic cost matrix for QP
    xlen = size(H,1);
	f = zeros(xlen,1);      % min (1/2p^TQp + f^Tp), linear cost matrix for QP

end