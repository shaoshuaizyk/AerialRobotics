function [A, b, newPath] = SCMFromPath(path, point_cloud_obs, map_boundary)
% LargeConvexPolytopes generates A and b from a single point, SCMFrom path
% generate A and b from a path.
    map_size = min(abs(map_boundary.ld - map_boundary.ru));
    tolerance = 1e-6;
    numSegments = size(path, 1) - 1;  % number of segments

    % preparation
    A = {};
    b = {};
    newPath = {};
    iieq = 0;  % count for number of the cell array

    for ii = 1:numSegments
        cur_pt = path(ii, :);
        next_pt = path(ii+1, :);

        segLength = norm(cur_pt - next_pt);
        R = segLength / 2;

        [cur_A, cur_b] = LargeConvexPolytopes(point_cloud_obs, cur_pt, R);

%         iieq = iieq + 1;
%         A{iieq} = cur_A;
%         b{iieq} = cur_b;
%         newPath{iieq} = cur_pt;

        % checking if the next positional point is outside the polygon 
        while any(cur_A * next_pt' >= cur_b)

            % raise error if the current point is not in the polygon
            if any(cur_A * cur_pt' >= cur_b)
                error("Wrong! Current point is not in the polytope!");
            end

            start = 0;
            goal = 1;
            distance = norm(next_pt - cur_pt);

            while distance * (goal - start) > tolerance
                mid = start + (goal - start) / 2;
                mid_pt = cur_pt + (next_pt - cur_pt) * mid; 

                if any(cur_A * mid_pt' >= cur_b)  % if midpoint is out
                    goal = mid;
                else
                    start = mid;
                end
            end

            cur_pt = cur_pt + (next_pt - cur_pt) * goal;  % update the current point

            [cur_A, cur_b] = LargeConvexPolytopes(point_cloud_obs, cur_pt, R);
            
            if any(cur_A * cur_pt' >= cur_b)
                error("Wrong! Current point is not in the polytope!");
            end

            iieq = iieq + 1;
            A{iieq} = cur_A;
            b{iieq} = cur_b;
            newPath{iieq} = cur_pt;

        end % end while

    end

    % add the last point
    iieq = iieq + 1;
    cur_pt = path(end, :);
    newPath{iieq} = cur_pt;

    % repacking the cell arraies
    newPathArr = zeros(iieq, 3);
    for ii = 1:iieq
        newPathArr(ii, :) = newPath{ii};
    end
    newPath = newPathArr;


end