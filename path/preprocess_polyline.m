function path = preprocess_polyline(points, closed)
%PREPROCESS_POLYLINE Validate CL points and compute segment metadata.
arguments
    points (:,:) double
    closed (1,1) logical = false
end
if size(points,2) == 2
    points(:,3) = 0;
end
if size(points,2) ~= 3 || size(points,1) < 2 || any(~isfinite(points), 'all')
    error('POP_MOO:InvalidPath', 'Path must be a finite N-by-2 or N-by-3 array.');
end
if closed && norm(points(1,:) - points(end,:)) > 1e-12
    points(end+1,:) = points(1,:);
end
delta = diff(points,1,1);
lengths = vecnorm(delta,2,2);
if any(lengths <= 1e-12)
    error('POP_MOO:DegenerateSegment', 'Consecutive CL points must be distinct.');
end
path.points = points;
path.closed = closed;
path.n_segments = numel(lengths);
path.segment = compute_segment_geometry(points);
path.lengths = lengths;
path.cumulative_s = [0; cumsum(lengths)];
path.total_length = sum(lengths);
path.dimension = 3;
% Every segment junction is a corner; for a closed path, 0 and L coincide.
path.corner_s = path.cumulative_s;
end
