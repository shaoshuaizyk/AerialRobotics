%{
Function to compute Aeq x = beq based on the minimum snap assumption and
the continuity (or corresponding derivatives) requirement for the
polynomial equations.

Input:


%}

function [Aeq, beq, elapsedTime] = PolynomialEquality(traj, ts, dim, n_poly, n_order)
    tic
    % unpacking
	p0 = traj.p0;
	pe = traj.pe;
    v0 = traj.v0;
    ve = traj.ve;
    a0 = traj.a0;
    ae = traj.ae;

    % compute Aeq x = beq 
    % continuous equality for polynomial equations
    % beacuse Aeq_x == Aeq_y == Aeq_z
    % Aeq = [Aeq_x  0      0]
    %       [0    Aeq_y    0]
    %       [0      0  Aeq_z]
    n_coef = n_order + 1;
	neq = 8;  %% (8 equations) for the starting and the ending points
    eqNum_x = (7*(n_poly-1)+neq);
	Aeq_x = zeros(eqNum_x, n_coef*n_poly);
	beq = zeros(dim*eqNum_x, 1);
	
	% start/terminal pva constraints  (8 equations)
	Aeq_x(1:4, 1:n_coef) = ...  % starting
                        [calc_dc(ts(1),n_order,0);
	                     calc_dc(ts(1),n_order,1);
	                     calc_dc(ts(1),n_order,2);
	                     calc_dc(ts(1),n_order,3)];
	Aeq_x(5:8, n_coef*(n_poly-1)+1:n_coef*n_poly) = ...  % ending
	                    [calc_dc(ts(end),n_order,0);
	                     calc_dc(ts(end),n_order,1);
	                     calc_dc(ts(end),n_order,2);
	                     calc_dc(ts(end),n_order,3)];
	beq(1:8,1)                              = [p0(1),v0(1),a0(1),0,pe(1),ve(1),ae(1),0]';
    beq((eqNum_x   + 1):(eqNum_x   + 8),1)  = [p0(2),v0(2),a0(2),0,pe(2),ve(2),ae(2),0]';
    beq((eqNum_x*2 + 1):(eqNum_x*2 + 8),1)  = [p0(3),v0(3),a0(3),0,pe(3),ve(3),ae(3),0]';
	
	% continuous constraints  ((n_poly-1)*7 equations)
	for i = 1:(n_poly-1)
		t_derc_p = calc_dc(ts(i+1),n_order,0);  % position
        t_derc_v = calc_dc(ts(i+1),n_order,1);  % velocity
        t_derc_a = calc_dc(ts(i+1),n_order,2);  % acceleration
        t_derc_j = calc_dc(ts(i+1),n_order,3);  % jerk
        t_derc_4 = calc_dc(ts(i+1),n_order,4);
        t_derc_5 = calc_dc(ts(i+1),n_order,5);
        t_derc_6 = calc_dc(ts(i+1),n_order,6);
        Aeq_x(neq+1, n_coef*(i-1)+1:n_coef*(i+1)) = [t_derc_p, -t_derc_p];
        Aeq_x(neq+2, n_coef*(i-1)+1:n_coef*(i+1)) = [t_derc_v, -t_derc_v];
        Aeq_x(neq+3, n_coef*(i-1)+1:n_coef*(i+1)) = [t_derc_a, -t_derc_a];
        Aeq_x(neq+4, n_coef*(i-1)+1:n_coef*(i+1)) = [t_derc_j, -t_derc_j];
        Aeq_x(neq+5, n_coef*(i-1)+1:n_coef*(i+1)) = [t_derc_4, -t_derc_4];
        Aeq_x(neq+6, n_coef*(i-1)+1:n_coef*(i+1)) = [t_derc_5, -t_derc_5];
        Aeq_x(neq+7, n_coef*(i-1)+1:n_coef*(i+1)) = [t_derc_6, -t_derc_6];
        neq = neq + 7;
    end
    
    zeroM = zeros(size(Aeq_x)); 
    Aeq = [Aeq_x, zeroM, zeroM; zeroM, Aeq_x, zeroM; zeroM, zeroM, Aeq_x];
    
    elapsedTime = toc;

end