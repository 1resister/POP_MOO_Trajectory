function [distance, closest, segment_index, segment_tau] = point_to_polyline_distance(points, polyline)
%POINT_TO_POLYLINE_DISTANCE True minimum distance to the union of CL segments.
if isstruct(polyline)
    vertices = polyline.points;
else
    vertices = polyline;
end
if size(points,2) == 2 && size(vertices,2) == 3
    points(:,3) = 0;
elseif size(points,2) == 3 && size(vertices,2) == 2
    vertices(:,3) = 0;
end
nPoint = size(points,1);
nSeg = size(vertices,1)-1;
allDistance = inf(nPoint,nSeg);
allTau = zeros(nPoint,nSeg);
allClosest = zeros(nPoint,size(points,2),nSeg);
for i = 1:nSeg
    [allDistance(:,i), allClosest(:,:,i), allTau(:,i)] = ...
        point_to_segment_distance(points, vertices(i,:), vertices(i+1,:));
end
[distance,segment_index] = min(allDistance,[],2);
closest = zeros(size(points));
segment_tau = zeros(nPoint,1);
for k = 1:nPoint
    closest(k,:) = allClosest(k,:,segment_index(k));
    segment_tau(k) = allTau(k,segment_index(k));
end
end
