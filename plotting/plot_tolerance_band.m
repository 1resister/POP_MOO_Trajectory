function hBand = plot_tolerance_band(ax,path,cfg)
%PLOT_TOLERANCE_BAND Draw the spatial dynamic contour-tolerance corridor.
faceColor=[1.00 0.78 0.16]; edgeColor=[0.82 0.52 0.05];
hBand=gobjects(1,1);
for segmentIndex=1:path.n_segments
    sampleCount=max(41,ceil(path.lengths(segmentIndex)/0.04)+1);
    tau=linspace(0,1,sampleCount).';
    p0=path.points(segmentIndex,1:2);
    p1=path.points(segmentIndex+1,1:2);
    centre=p0+(p1-p0).*tau;
    progress=path.cumulative_s(segmentIndex)+tau*path.lengths(segmentIndex);
    region=classify_corner_regions(progress,path,cfg);
    epsilon=region.allowed_error(:);
    normal=path.segment(segmentIndex).normal(1:2);
    upper=centre+epsilon.*normal;
    lower=centre-epsilon.*normal;
    x=[upper(:,1);flipud(lower(:,1))];
    y=[upper(:,2);flipud(lower(:,2))];
    if segmentIndex==1
        hBand=patch(ax,x,y,faceColor,'FaceAlpha',0.22,'EdgeColor','none', ...
            'DisplayName','Allowable tolerance band');
    else
        patch(ax,x,y,faceColor,'FaceAlpha',0.22,'EdgeColor','none', ...
            'HandleVisibility','off');
    end
    plot(ax,upper(:,1),upper(:,2),':','Color',edgeColor,'LineWidth',0.9, ...
        'HandleVisibility','off');
    plot(ax,lower(:,1),lower(:,2),':','Color',edgeColor,'LineWidth',0.9, ...
        'HandleVisibility','off');
end

% Complete the tolerance tube at vertices using the corner tolerance.
if path.closed
    vertices=path.points(1:end-1,1:2);
else
    vertices=path.points(:,1:2);
end
theta=linspace(0,2*pi,81);
radius=cfg.geometry.corner_tolerance;
for vertexIndex=1:size(vertices,1)
    x=vertices(vertexIndex,1)+radius*cos(theta);
    y=vertices(vertexIndex,2)+radius*sin(theta);
    patch(ax,x,y,faceColor,'FaceAlpha',0.22,'EdgeColor','none', ...
        'HandleVisibility','off');
    plot(ax,x,y,':','Color',edgeColor,'LineWidth',0.9,'HandleVisibility','off');
end
end
