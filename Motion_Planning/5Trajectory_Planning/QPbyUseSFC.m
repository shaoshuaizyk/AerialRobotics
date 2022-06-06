function [X, ElapsedTimes] = QPbyUseSFC(waypts, ts, decomp)

%% condition
traj.n_order = 7;
traj.n_poly = size(waypts, 1) - 1;  % one less than num of waypoints
traj.p0 = waypts(1,:);
traj.pe = waypts(end,:);
traj.v0 = [0,0,0];
traj.ve = [0,0,0];
traj.a0 = [0,0,0];
traj.ae = [0,0,0];


%% trajectory plan
[minSnapValue, px, py, pz, ElapsedTimes] = minimum_snap_three_axis_SFC(traj, ts, decomp);

disp(['minSnapValue is : ',num2str(minSnapValue)]);

X = [px py pz];
end

% v0 = [1*3]
function [minValue, px, py, pz, ElapsedTimes] = minimum_snap_three_axis_SFC(traj, ts, decomp)

    % dimension and parameters
    dim = 3;
    n_order = traj.n_order;
    n_poly = traj.n_poly;

    % compute Q and f cost
    [Q_all, f] = MinSnapQPCost(ts, n_order, n_poly);
    xlen = size(Q_all, 1);
	
    % compute Aeq x = beq
    [Aeq, beq, AbeqTime] = PolynomialEquality(traj, ts, dim, n_poly, n_order);

    % compute Ax <= b
    [A, b, AbTime] = PolygonInequality(decomp, ts, dim, n_poly, n_order, xlen);
    
    % Modify the number of iterations
    options = optimoptions('fmincon');
    options.MaxIterations = 2000;   % Defaults = 1000
    options.Diagnostics = "on";
    lb = []; ub = []; x0 = []; 
    tic
    p = quadprog(Q_all, f, A, b, Aeq, beq, lb, ub, x0, options);
    QPTime = toc;
    minValue = p' * Q_all * p;      % minimum value for cost
	
    xll = xlen / dim;
    px = p(1:xll, 1);           % x's
    py = p(xll+1:2*xll, 1);     % y's
    pz = p(2*xll+1:3*xll, 1);   % z's

    % returning time consumed
    ElapsedTimes = struct(...
        'polytope', AbTime,...
        'polynomial', AbeqTime,...
        'QP', QPTime,...
        'total', sum([AbTime, AbeqTime, QPTime]'));
end

