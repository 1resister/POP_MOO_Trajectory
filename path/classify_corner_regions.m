function region = classify_corner_regions(path_progress, path, cfg)
%CLASSIFY_CORNER_REGIONS Locate samples by arc-distance from path corners.
s = path_progress(:);
cornerS = path.corner_s(:).';
distanceMatrix = abs(s-cornerS);
if path.closed
    distanceMatrix = min(distanceMatrix, path.total_length-distanceMatrix);
end
[dCorner, cornerIndex] = min(distanceMatrix,[],2);
region.distance_to_corner = dCorner;
region.corner_index = cornerIndex;
region.is_corner = dCorner < cfg.geometry.transition_length;
region.is_line_core = ~region.is_corner;
region.allowed_error = dynamic_tolerance(dCorner,cfg);
end
