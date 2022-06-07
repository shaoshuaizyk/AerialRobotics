%{
Draw surfaces centered around a point "ct" described by the Ax = b
equation. 

Input:
    As: cell array, containing A's from each polytope
    bs: cell array, containing b's from each polytope
    wps: (n+1)x3 vector, waypoints considered
    bound: struct, map boundaries

Output:
    <None>

%}

function [] = DrawPolyFromAbCells(As, bs, wps, bound)
    
    % parameters
    %lsNum = 2;  % all surfaces are flat so only 2 is needed
    %offSet = 0.5;  % determines how large we want the surface to be shown
    tol = 1e-9;
    alphaNum = 0.1;  % transparency

    % number of polytopes
    numPoly = length(As);

    % boundaries
    ld = bound.ld;
    ru = bound.ru;
    xl = [ld(1), ru(1)];
    yl = [ld(2), ru(2)];
    zl = [ld(3), ru(3)];

    % draw
    figure('Position', [10, 10, 1200, 800])
    hold on
    for pp = 1:numPoly
        % unpacking
        A = As{pp};
        b = bs{pp};
        cp = wps(pp, :) + 0.5 * (wps(pp + 1, :) - wps(pp, :));

        % how many surfaces
        numSurf = length(b);
        numIntPoints = 0;
        for ii = 1:numSurf-2
            numIntPoints = numIntPoints + sum(1:ii);
        end
        intPoints = zeros(3, numIntPoints);
        count = 2;

        for ii = 1:numSurf-2
            for jj = ii+1:numSurf-1
                for kk = jj+1:numSurf
                    A_cur = [A(ii, :); A(jj, :); A(kk, :)];
                    b_cur = [b(ii); b(jj); b(kk)];
                    if det(A_cur) > tol  % have a good unique solution        
                        count = count + 1;
                        intPoints(:, count) = A_cur \ b_cur;
                    end
                end
            end
        end

        % remove the "bad" solutions
        intPoints = intPoints(:, 1:count);
        [~, numIntPoints] = size(intPoints);
        
        % plot surfaces based on points
        for ii = 1:numIntPoints-2
            for jj = ii+1:numIntPoints-1
                for kk = jj+1:numIntPoints
                    pt_1 = intPoints(:, ii);
                    pt_2 = intPoints(:, jj);
                    pt_3 = intPoints(:, kk);

                    h = patch('Faces',1:3,'Vertices',[pt_1, pt_2, pt_3]');
                    set(h, 'FaceColor', 'b', 'EdgeColor', 'b', 'LineWidth', 1,...
                            'FaceAlpha', alphaNum, 'EdgeAlpha', alphaNum);
    
                end
            end
        end

    end

    % draw paths with different color
    rVals = linspace(0, 1, numPoly);
    gVals = linspace(1, 0, numPoly);
    bVals = linspace(0, 0, numPoly);

    for ii = 1:numPoly
        curSeg = wps(ii:ii+1, :);
        curRGB = [rVals(ii), gVals(ii), bVals(ii)];
        plot3(curSeg(:,1), curSeg(:,2), curSeg(:,3), 'Color', curRGB, 'LineWidth', 16)
    end

    hold off
    view([1, 1, 1])
    xlim(xl)
    ylim(yl)
    zlim(zl)
    xlabel('x', 'FontSize', 20)
    ylabel('y', 'FontSize', 20)
    zlabel('z', 'FontSize', 20)


end










%         xRange = linspace(ct(1) - offSet, ct(1) + offSet, lsNum);
%         yRange = linspace(ct(2) - offSet, ct(2) + offSet, lsNum);
%         [X, Y] = meshgrid(xRange, yRange);
%         Z = zeros(size(X));
% 
%         for ss = 1:numSurf
%             ARow = A(ss, :);
%             bRow = b(ss);
% 
%             for ii = 1:lsNum
%                 for jj = 1:lsNum
%                     Z(ii, jj) = CalcZ(X(ii, jj), Y(ii, jj), ARow, bRow);
%                 end
%             end
%             surface(X, Y, Z, 'FaceAlpha', 0.3);
%         end
%     end
%     
%     % plot path with different color
%     rVals = linspace(0, 1, numPoly);
%     gVals = linspace(1, 0, numPoly);
%     bVals = linspace(0, 0, numPoly);
% 
%     for ii = 1:numPoly
%         curSeg = cts(ii:ii+1, :);
%         curRGB = [rVals(ii), gVals(ii), bVals(ii)];
%         plot3(curSeg(:,1), curSeg(:,2), curSeg(:,3), 'Color', curRGB, 'LineWidth', 16)
%     end
%     hold off
%     view([1, 1, 1]);
%     xlim(xl);
%     ylim(yl);
%     zlim(zl);
% 
% end
% 
% 
% % given x, y, and A b, calculate z
% function [z] = CalcZ(x, y, A, b)
% 
%     z = (1 / A(3)) * (b - A(1) * x - A(2) * y);
% 
% end