function metrics = calculate_contour_error(solution,path,cfg)
%CALCULATE_CONTOUR_ERROR True Euclidean error to the whole finite polyline.
points=[solution.position.',zeros(solution.N+1,1)];
[error,closest,segment,tau]=point_to_polyline_distance(points,path);
s=zeros(size(error));
for k=1:numel(error)
    s(k)=path.cumulative_s(segment(k))+tau(k)*path.lengths(segment(k));
end
region=classify_corner_regions(s,path,cfg);
allowed=region.allowed_error;
utilisation=error./max(allowed,eps);
metrics.error=error; metrics.closest=closest; metrics.segment=segment;
metrics.segment_tau=tau; metrics.path_progress=s;
metrics.allowed_error=allowed; metrics.utilisation=utilisation;
metrics.maximum=max(error); metrics.rmse=sqrt(mean(error.^2));
if any(region.is_line_core)
    metrics.line_max=max(error(region.is_line_core));
    metrics.line_utilisation_mean=mean(utilisation(region.is_line_core));
else
    metrics.line_max=NaN; metrics.line_utilisation_mean=NaN;
end
if any(region.is_corner)
    metrics.corner_max=max(error(region.is_corner));
    metrics.corner_utilisation_mean=mean(utilisation(region.is_corner));
else
    metrics.corner_max=NaN; metrics.corner_utilisation_mean=NaN;
end
metrics.maximum_utilisation=max(utilisation);
metrics.mean_utilisation=mean(utilisation);
metrics.region=region;
end
