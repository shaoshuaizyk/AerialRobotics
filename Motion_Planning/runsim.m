% if demoActivate exist and equal true, not clean all variable.
% runsim.m called by the demo,  not clean all variable
% runsim.m called by itself,  clean all variable
if (~exist('demoActivate')) || (demoActivate == false)
    close all;
    clear all;
    clc;
end

addpath(genpath('./'));


%% parameters
converts2ms = 1000;
avgTimeRuns = 1;
ratioNum = 31;
figPlot = false;
noFillPts = false;
mode_1 = true;
mode_2 = false;
mode_3 = false;
skipQP = true;

%% map generation
map = GridMap();
init_data
map.flag_obstacles();


%% path planning
nquad = length(start);

if mode_1

    JPSTime = 0;
    JPS2SFCTime = 0;
    
    SFCTime.polytope = 0;
    SFCTime.polynomial = 0;
    SFCTime.totalWoQP = 0;
    SFCTime.QP = 0;
    SFCTime.total = 0;
    
    SCPTime.polytope = 0;
    SCPTime.polynomial = 0;
    SCPTime.totalWoQP = 0;
    SCPTime.QP = 0;
    SCPTime.total = 0;

    for ii = 1:avgTimeRuns

        tic
        for qn = 1:nquad    
            path{qn} = JPS_3D(map, start{qn}, stop{qn});
            path{qn} = [start{qn}; path{qn}(1:end,:)];
            path{qn}(end + 1,:) = stop{qn};
        end
        JPSTimeCur = toc;
        JPSTime = JPSTime + JPSTimeCur;
        
        % delete the points of no-use
        path{2} = simplify_path(map, path{1});
        
        % Generating Convex Polytopes
        obps = PointCloudMap(map.blocks, map.margin);   % blocks of Metric Map change to point cloud
        
        decomps{2} = [];
        
        % main 1
        tic
        decomps{1} = SFC_3D(path{2}, obps, map.boundary); %  call SFC
        JPS2SFCTimeCur = toc;
        JPS2SFCTime = JPS2SFCTime + JPS2SFCTimeCur;    
    
        % Trajectory planning (Liu)
        [t_time, ts_par, x_par, SFCTimes] = TrajectoryPlanning(path{path_id}, decomps{SFC_id}, time_allocation);
        SFCTime.polytope = SFCTime.polytope + SFCTimes.polytope;
        SFCTime.polynomial = SFCTime.polynomial + SFCTimes.polynomial;
        SFCTime.totalWoQP = SFCTime.totalWoQP + SFCTimes.polytope + SFCTimes.polynomial;
        SFCTime.QP = SFCTime.QP + SFCTimes.QP;
        SFCTime.total = SFCTime.total + SFCTimes.total;
        
        % Trajectory planning (star-convex)   
        ratioR = 0.5;
        [X, SCPTimes, ~, ~, ~] = StarConvexTrajectoryPlanning(path{path_id}, obps, map.boundary, time_allocation, ratioR, figPlot, noFillPts, skipQP);
        SCPTime.polytope = SCPTime.polytope + SCPTimes.polytope;
        SCPTime.polynomial = SCPTime.polynomial + SCPTimes.polynomial;
        SCPTime.totalWoQP = SCPTime.totalWoQP + SCPTimes.polytope + SCPTimes.polynomial;
        SCPTime.QP = SCPTime.QP + SCPTimes.QP;
        SCPTime.total = SCPTime.total + SCPTimes.total;
        
    end

    % adjusting times
    JPSTime = JPSTime / avgTimeRuns;
    JPS2SFCTime = JPS2SFCTime / avgTimeRuns;
    
    SFCTime.polytope = SFCTime.polytope / avgTimeRuns;
    SFCTime.polynomial = SFCTime.polynomial / avgTimeRuns;
    SFCTime.totalWoQP = SFCTime.totalWoQP / avgTimeRuns;
    SFCTime.QP = SFCTime.QP / avgTimeRuns;
    SFCTime.total = SFCTime.total / avgTimeRuns;
    
    SCPTime.polytope = SCPTime.polytope / avgTimeRuns;
    SCPTime.polynomial = SCPTime.polynomial / avgTimeRuns;
    SCPTime.totalWoQP = SCPTime.totalWoQP  / avgTimeRuns;
    SCPTime.QP = SCPTime.QP / avgTimeRuns;
    SCPTime.total = SCPTime.total / avgTimeRuns;
    
    % reporting elapsed times
    fprintf("\n\n")
    fprintf("All time reported are averaged times\n")
    fprintf("JPS init path elapsed time: %.6f ms\n", JPSTime * converts2ms)
    fprintf("\n\n")
    fprintf("JPS -> SFC path elapsed time: %.6f ms\n", JPS2SFCTime * converts2ms)
    fprintf("\n\n")
    fprintf('SFC -> SFC Trajectory polytope time is:           %.6f ms\n', SFCTime.polytope * converts2ms)
    fprintf('SFC -> SFC Trajectory polynomial time is:         %.6f ms\n', SFCTime.polynomial * converts2ms)
    fprintf('SFC -> SFC Trajectory p+p w/o quadprog time is:   %.6f ms\n', SFCTime.totalWoQP * converts2ms)
    fprintf('SFC -> SFC Trajectory quadprog time is:           %.6f ms\n', SFCTime.QP * converts2ms)
    fprintf("\n\n")
    fprintf('JPS -> SFC Trajectory polytope time is:           %.6f ms\n', (SFCTime.polytope + JPS2SFCTime) * converts2ms)
    fprintf('JPS -> SFC Trajectory polynomial time is:         %.6f ms\n', SFCTime.polynomial * converts2ms)
    fprintf('JPS -> SFC Trajectory quadprog time is:           %.6f ms\n', SFCTime.QP * converts2ms)
    fprintf('JPS -> SFC Trajectory total w/o quadprog time is: %.6f ms\n', (SFCTime.totalWoQP + JPS2SFCTime) * converts2ms)
    fprintf('JPS -> SFC Trajectory total time is:              %.6f ms\n', (SFCTime.total + JPS2SFCTime) * converts2ms)
    fprintf("\n\n")
    fprintf('JPS -> StarConvex Trajectory polytope time is :              %.6f ms\n', SCPTime.polytope * converts2ms)
    fprintf('JPS -> StarConvex Trajectory polynomial time is :            %.6f ms\n', SCPTime.polynomial * converts2ms)
    if not(skipQP)
        fprintf('JPS -> StarConvex Trajectory quadprog time is :              %.6f ms\n', SCPTime.QP * converts2ms)
    end
    fprintf('JPS -> StarConvex Trajectory total p+p w/o quadprog time is: %.6f ms\n', SCPTime.totalWoQP * converts2ms)
    fprintf('JPS -> SFC Trajectory total time is:                         %.6f ms\n', SCPTime.total * converts2ms)
    fprintf("\n\n")
end

%% relationship between R (map size) and polytope time & number of polytopes & number of surfaces
if mode_2

    tic
    for qn = 1:nquad    
        path{qn} = JPS_3D(map, start{qn}, stop{qn});
        path{qn} = [start{qn}; path{qn}(1:end,:)];
        path{qn}(end + 1,:) = stop{qn};
    end
    JPSTimeCur = toc;
    JPSTime = JPSTime + JPSTimeCur;
    
    
    
    % delete the points of no-use
    path{2} = simplify_path(map, path{1});
    
    % Generating Convex Polytopes
    obps = PointCloudMap(map.blocks, map.margin);   % blocks of Metric Map change to point cloud
    
    decomps{2} = [];


    ratioArr = logspace(-1, 1, ratioNum);
    RArr = zeros(size(ratioArr));
    polytopeTimeArr = zeros(size(ratioArr));
    polynomialTimeArr = zeros(size(ratioArr));
    numPolyArr = zeros(size(ratioArr));
    avgSurfArr = zeros(size(ratioArr));
    for ii = 1:ratioNum
        ratioR = ratioArr(ii);
        SCPTime.polytope = 0;
        SCPTime.polynomial = 0;
        numPoly = 0;
        numSurf = 0;
        for jj = 1:avgTimeRuns
            [~, SCPTimes, RArr(ii), curPoly, curSurf] = StarConvexTrajectoryPlanning(path{path_id}, obps, map.boundary, time_allocation, ratioR, figPlot, noFillPts);
            SCPTime.polytope = SCPTime.polytope + SCPTimes.polytope;
            SCPTime.polynomial = SCPTime.polynomial + SCPTimes.polynomial;
            numPoly = numPoly + curPoly;
            numSurf = numSurf + curSurf;
        end
        polytopeTimeArr(ii) = SCPTime.polytope / avgTimeRuns * converts2ms;
        polynomialTimeArr(ii) = SCPTime.polynomial / avgTimeRuns * converts2ms;
        numPolyArr(ii) = numPoly / avgTimeRuns;
        avgSurfArr(ii) = numSurf / avgTimeRuns;
        avgSurfArr(ii) = avgSurfArr(ii) / numPolyArr(ii);
    end
    
    % plots
    figure('Position', [10, 10, 1200, 800])
    yyaxis left
    hold on
    loglog(ratioArr, polytopeTimeArr, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 1, 'DisplayName', 'Polyhedra Time')
    loglog(ratioArr, polynomialTimeArr, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'Polynomial Time')
    hold off
    ylabel("Time in (ms)", 'FontSize', 20, 'Interpreter', 'latex')
    yyaxis right
    hold on
    loglog(ratioArr, numPolyArr, 'Color', 'r', 'LineStyle', '-', 'LineWidth', 1, 'DisplayName', 'Number of Polyhedra')
    loglog(ratioArr, avgSurfArr, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'Average Number of Surfaces')
    hold off
    ylabel("Numbers", 'FontSize', 20, 'Interpreter', 'latex')
    xlabel("R as ratio of map size", 'FontSize', 20, 'Interpreter', 'latex')
    title("Results on the effect of radius R (based on map size)", 'FontSize', 24, 'Interpreter', 'latex')
    set(gca, 'XScale', 'log')
    grid on
    legend("Location", 'northeast', 'FontSize', 16)
end
    
%% relationship between R (segment length) and polytope time & number of polytopes & number of surfaces
if mode_3
    ratioArr = logspace(-1, 1, ratioNum) * -1;
    RArr = zeros(size(ratioArr));
    polytopeTimeArr = zeros(size(ratioArr));
    polynomialTimeArr = zeros(size(ratioArr));
    numPolyArr = zeros(size(ratioArr));
    avgSurfArr = zeros(size(ratioArr));
    for ii = 1:ratioNum
        ratioR = ratioArr(ii);
        SCPTime.polytope = 0;
        SCPTime.polynomial = 0;
        numPoly = 0;
        numSurf = 0;
        for jj = 1:avgTimeRuns
            [~, SCPTimes, RArr(ii), curPoly, curSurf] = StarConvexTrajectoryPlanning(path{path_id}, obps, map.boundary, time_allocation, ratioR, figPlot, noFillPts);
            SCPTime.polytope = SCPTime.polytope + SCPTimes.polytope;
            SCPTime.polynomial = SCPTime.polynomial + SCPTimes.polynomial;
            numPoly = numPoly + curPoly;
            numSurf = numSurf + curSurf;
        end
        polytopeTimeArr(ii) = SCPTime.polytope / avgTimeRuns * converts2ms;
        polynomialTimeArr(ii) = SCPTime.polynomial / avgTimeRuns * converts2ms;
        numPolyArr(ii) = numPoly / avgTimeRuns;
        avgSurfArr(ii) = numSurf / avgTimeRuns;
        avgSurfArr(ii) = avgSurfArr(ii) / numPolyArr(ii);
    end
    
    % plots
    ratioArr = ratioArr * -1;
    figure('Position', [10, 10, 1200, 800])
    yyaxis left
    hold on
    loglog(ratioArr, polytopeTimeArr, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 1, 'DisplayName', 'Polyhedra Time')
    loglog(ratioArr, polynomialTimeArr, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'Polynomial Time')
    hold off
    ylabel("Time in (ms)", 'FontSize', 20, 'Interpreter', 'latex')
    yyaxis right
    hold on
    loglog(ratioArr, numPolyArr, 'Color', 'r', 'LineStyle', '-', 'LineWidth', 1, 'DisplayName', 'Number of Polyhedra')
    loglog(ratioArr, avgSurfArr, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'Average Number of Surfaces')
    hold off
    ylabel("Numbers", 'FontSize', 20, 'Interpreter', 'latex')
    xlabel("R as ratio of segment lengths", 'FontSize', 20, 'Interpreter', 'latex')
    title("Results on the effect of radius R (based on segment lengths)", 'FontSize', 24, 'Interpreter', 'latex')
    set(gca, 'XScale', 'log')
    grid on
    legend("Location", 'northeast', 'FontSize', 16)
end


%% draw path and blocks
if nquad == 1
    plot_path(path, map, decomps); 
else
    % you could modify your plot_path to handle cell input for multiple robots
end

% draw_ObcPoints
% makeGifAndJpg(1);     %figure(1): Graph without trajectory


%% Trajectory tracking
disp('Generating Trajectory ...');
trajectory_generator([], [], path{path_id}, t_time, ts_par, x_par);
trajectory = test_trajectory(start, stop, path, true); % with visualization
disp('Blue line is Trajectory planning.');
disp('Red line is Trajectory tracking.');

%% Gif
% makeGifAndJpg(3);     %figure(3): Graph with trajectory
