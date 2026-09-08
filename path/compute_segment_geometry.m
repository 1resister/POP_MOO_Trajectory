function segment = compute_segment_geometry(points)
%COMPUTE_SEGMENT_GEOMETRY Tangent/normal/local progress for each CL segment.
n = size(points,1)-1;
segment = repmat(struct('p0',[],'p1',[],'vector',[],'length',[], ...
    'tangent',[],'normal',[]), n, 1);
for i = 1:n
    d = points(i+1,:) - points(i,:);
    L = norm(d);
    t = d/L;
    if abs(t(3)) < 1e-12
        normal = [-t(2), t(1), 0];
    else
        normal = [NaN NaN NaN]; % a 3-D segment has a normal plane, not one normal
    end
    segment(i).p0 = points(i,:);
    segment(i).p1 = points(i+1,:);
    segment(i).vector = d;
    segment(i).length = L;
    segment(i).tangent = t;
    segment(i).normal = normal;
end
end
