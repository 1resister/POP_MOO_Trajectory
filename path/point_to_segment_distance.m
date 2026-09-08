function [distance, closest, tau] = point_to_segment_distance(points, p0, p1)
%POINT_TO_SEGMENT_DISTANCE Exact Euclidean distance to a finite segment.
% points is N-by-D; tau is clamped fractional progress in [0,1].
if isvector(points)
    points = reshape(points,1,[]);
end
p0 = reshape(p0,1,[]);
p1 = reshape(p1,1,[]);
d = p1-p0;
den = dot(d,d);
if den <= eps
    error('POP_MOO:DegenerateSegment', 'Segment endpoints must differ.');
end
tau = ((points-p0)*d.')/den;
tau = min(1,max(0,tau));
closest = p0 + tau.*d;
distance = vecnorm(points-closest,2,2);
end
