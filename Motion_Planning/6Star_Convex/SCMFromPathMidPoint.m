%{
Function to generate the polytopes (represented by the inequality Ax <= b)
using the star convex method from the adjusted midpoints of the path.

%}


function [A, b, newPath] = SCMFromPathMidPoint(path, ptCloudObs, mapBound)
% LargeConvexPolytopes generates A and b from a single point, SCMFrom path
% generate A and b from a path.
    map_size = min(abs(mapBound.ld - mapBound.ru));
    tolerance = 1e-6;
    numSegments = size(path, 1) - 1;  % number of segments

    % preparation
    As = {};
    bs = {};
    newPath = {};
    iieq = 0;  % count for number of the cell array

    % midpoint method
    for ii = 1:numSegments
        startPoint = path(ii, :);
        endPoint = path(ii+1, :);

        % calculate midpoint
        segLen = norm(endPoint - startPoint);
        R = segLen;  % setting R = segment length
        midPoint = startPoint + 0.5 * (endPoint - startPoint);

        % generate Ab from midpoint
        [A, b] = LargeConvexPolytopes(ptCloudObs, midPoint, R);

        % check if startPoint is excluded from the polytope
        startOut = any(A * startPoint' >= b);

        % check if endPoint is excluded from the polytope
        endOut = any(A * endPoint' >= b);

        % if start point is excluded, turn the current midPoint to the
        % new endPoint and continue until satisfied
        if startOut
            

        end







    end




    for ii = 1:numSegments
        cur_pt = path(ii, :);
        next_pt = path(ii+1, :);

        segment_length = norm(cur_pt - next_pt);
        R = map_size;

        [cur_A, cur_b] = LargeConvexPolytopes(ptCloudObs, cur_pt, R);

        iieq = iieq + 1;
        A{iieq} = cur_A;
        b{iieq} = cur_b;
        newPath{iieq} = cur_pt;

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

                if any(cur_A * mid_pt' >= cur_b)  % if midpoint satisfied the condition
                    goal = mid;
                else
                    start = mid;
                end
            end

            cur_pt = cur_pt + (next_pt - cur_pt) * goal;  % update the current point

            [cur_A, cur_b] = LargeConvexPolytopes(ptCloudObs, cur_pt, R);
            
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


%% Resursive function to find A b based on the midpoint method
function [mps, As, bs] = FindAbRecursively(mps, As, bs, sp, ep, ii)
    
    % add one to count
    ii = ii + 1;

    % find midpoint
    mp = sp + 0.5 * (ep - sp);

    % find A and b from midpoint
    [A, b] = LargeConvexPolytopes(ptCloudObs, midPoint, R);

    % if startpoint is outside
    if any(A * sp' >= b)
        epnew = mp;  % set current midpoint to the new endpoint
        FindAbRecursively(mps, As, bs, sp, epnew, ii);

    % if endpoint is outside
    elseif any(A * ep' >= b)
        spnew = mp;  % set current midpoint to the new startpoint
        FindAbRecursively(mps, As, bs, spnew, ep, ii);

    % if all satisfied, exit the current recursion
    else
        % exit condition, packing parameters
    end

end