function test_contour_error()
vertices = [0 0; 2 0; 2 2];
points = [1 1; 3 1; 2 1; -1 0];
[d,~,idx] = point_to_polyline_distance(points,vertices);
assert(max(abs(d-[1;1;0;1]))<1e-12);
assert(isequal(idx,[1;2;2;1]));
fprintf('PASS test_contour_error\n');
end
