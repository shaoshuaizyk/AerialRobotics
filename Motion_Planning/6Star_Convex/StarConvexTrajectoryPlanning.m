%{
Trajectory planning using star convex method for safe flight corridor (SFC)
generation, and quadratic programming (QP) with minimum snap assumption for
optimization 

Inputs:
    wayPoints: nx3 array of waypoints (including start and end)
    pointCloudObs: nx3 array of object points coordinates
    mapBoundary: struct with field ld and ru, map boundaries

%}



function [X, ElapsedTimes, R, numPoly, numSurf] = StarConvexTrajectoryPlanning(wayPoints, pointCloudObs, mapBoundary, time_allocation, ratioR, figPlot, noFillPts)
    % dimension
    dim = 3;

    % generating densified polytopes satisfying the SFC connection
    [A, b, path, R, numPoly, numSurf] = SCMFromPath(wayPoints, pointCloudObs, mapBoundary, ratioR, noFillPts); 

    % drawing surface
    if figPlot
        DrawPolyFromAbCells(A(1:2), b(1:2), path(1:3, :), mapBoundary);
    end
%     segNum = length(b);
%     constNum = 0;
%     for ii = 1:segNum
%         constNum = constNum + length(b{ii});
%     end
%     constNum

    % time allocation
    speed   = time_allocation.avg_speed;
    acc     = time_allocation.acc;
    
%     disp(['The planned speed is : ', num2str(speed)]);
%     tic
    if strcmp(time_allocation.type, 'averageSpeed')
        [ts, total_time] = averageSpeed_ta(path, speed);
    elseif strcmp(time_allocation.type, 'trapzoidSpeed')
        [ts, total_time] = trapezoidalSpeed_ta(path, speed, acc);
    end
%     disp('TimeAllocation time is :');
%     toc
%     disp(['time management: total_time is ', num2str(total_time), ' seconds']);
%     disp(['Split time is : ', num2str(ts)]);

    % generating the cost matrices for minimum snap trajectory
    n_order = 7;  % min snap
    n_poly = length(A);
    [H, f] = MinSnapQPCost(ts, n_order, n_poly);
    xlen = length(H);

    % generating the equality constraints for minimum snap trajectory
    traj.p0 = path(1,:);
    traj.pe = path(end,:);
    traj.v0 = [0,0,0];
    traj.ve = [0,0,0];
    traj.a0 = [0,0,0];
    traj.ae = [0,0,0];
    [Aeq, beq, timePolynomial] = PolynomialEquality(traj, ts, dim, n_poly, n_order);

    % converting the A b cell arrays to point and vector representation for
    % planes
    [decomp] = ConvertCellAbToPointAndNormalVector(A, b, path);
    [A, b, timePolytope] = PolygonInequality(decomp, ts, dim, n_poly, n_order, xlen);

    % quadprog
    options = optimoptions('fmincon');
    options.MaxIterations = 10000;   % Defaults = 1000
    options.Diagnostics = "on";
    lb = []; ub = []; x0 = [];
    tic
%     p = quadprog(H, f, A, b, Aeq, beq, lb, ub, x0, options);
    QPTime = toc;
%     minValue = p' * H * p;      % minimum value for cost
%     disp(['minSnapValue is : ',num2str(minValue)]);
	
    xll = xlen / dim;
%     px = p(1:xll, 1);           % x's
%     py = p(xll+1:2*xll, 1);     % y's
%     pz = p(2*xll+1:3*xll, 1);   % z's

    % returning time consumed
    ElapsedTimes = struct(...
        'polytope', timePolytope,...
        'polynomial', timePolynomial,...
        'QP', QPTime,...
        'total', sum([timePolytope, timePolynomial, QPTime]'));

    % packing
%     X = [px, py, pz];
    X = [];


end