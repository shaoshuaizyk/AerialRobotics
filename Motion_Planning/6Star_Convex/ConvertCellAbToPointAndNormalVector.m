%{
Function to convert the cell arrays of A and b to matrices according to the
section 5 Trajectory_Planning with SFC (based on Liu's paper). Goal is to
use the QP the same way as in section 5.

Input:
    A, b: cell arrays, each segment has polygon of faces defined by Ax <= b

Output:
    decomp: struct, the same format used in SFC section 

%}


function [decomp] = ConvertCellAbToPointAndNormalVector(A, b, wayPoints)

    % parameters
    numPoly = length(A);  % number of polygons in total (segments)

    % pre-allocating the spaces
    decomp.lines_ = cell(1, numPoly);
    
    for ii = 1:numPoly
        ACur = A{ii};
        bCur = b{ii};
        [numPlanes, ~] = size(ACur);  % get number of planes in this polytope
        planesCell = cell(1, numPlanes);  % init cell array
        
        for jj = 1:numPlanes  % convert each plane into p_ and n_ format
            ARow = ACur(jj, :);
            ARow_pinv = pinv(ARow);
            bVal = bCur(jj);
            centerPoint = ARow_pinv * bVal;  % one point in the plane
            planesCell{jj}.p_ = centerPoint';
            planesCell{jj}.n_ = ARow';

        end

        % packing
        decomp.lines_{ii}.polyhedron_.polys_ = planesCell;

    end

end