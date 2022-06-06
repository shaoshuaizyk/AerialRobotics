%{
Function to compute Ax <= b based on the given polygons. It can be used as
a boundary inequality for quadratic programming

Input:
    decomp: struct, containing decompositions informations
    ts: array, time along n polygons
    dim: int, dimension of the system
    n_poly: int, number of polynomial equations needed (waypoints - 1) 
    n_order: int, highest power of the polynomial trajectory
    xlen: int, total number of decision variables (times with dimensions)

Output:
    A, b: matrices for the constrain Ax <= b

%}


function [A, b, elapsedTime] = PolygonInequality(decomp, ts, dim, n_poly, n_order, xlen)
    tic
    % boundary inequality for polygons
    % pre-allocating A and b (added by XT)
    numPlanes = 0;
    for ii = 1:n_poly
        planesCell = decomp.lines_{ii}.polyhedron_.polys_;
        [~, newNumPlanes] = size(planesCell);
        numPlanes = numPlanes + newNumPlanes;
    end
    A = zeros(dim * numPlanes, xlen);
    b = zeros(dim * numPlanes, 1);

    % filling out the entries
    % add (Denominator + 1) points on the poly's Ax<b 
    Denominator = 2;  % Denominator > 0
    ieq = 1;
    xll = xlen / dim;  % 1/3 of xlen,  = n_coef * (n_poly-1)
    for i = 1 : n_poly
        planesCell = decomp.lines_{i}.polyhedron_.polys_;
        [~, lenj] = size(planesCell);
        for j = 1 : lenj
            n = planesCell{j}.n_;
            p = planesCell{j}.p_;
            nx = zeros(1, xlen);  % x len is the size of Q
            
            for k = 0 : Denominator
                tvec_mid = calc_dc(ts(i) +  k*(ts(i+1) - ts(i))/Denominator, n_order, 0);   % [1*8]vector
                nx(1, (n_order+1)*(i-1) + 1 : (n_order+1)*(i-1) + (n_order+1))                    = tvec_mid.*n(1);
                nx(1, (n_order+1)*(i-1) + 1 + xll : (n_order+1)*(i-1) + (n_order+1) + xll)        = tvec_mid.*n(2);
                nx(1, (n_order+1)*(i-1) + 1 + 2*xll : (n_order+1)*(i-1) + (n_order+1) + 2*xll)    = tvec_mid.*n(3);
                A(ieq,:) = nx;
                b(ieq,:) = dot(n, p);
                % dot(plane.n, p1 - plane.p) < 0 
                % => dot(plane.n, p1) - dot(plane.n, plane.p) < 0 
                % => dot(plane.n, p1) < dot(plane.n, plane.p) 
                % Ax < b
                ieq = ieq + 1;
            end
        end
    end
    elapsedTime = toc;
end