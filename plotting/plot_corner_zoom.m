function plot_corner_zoom(path,solutions,labels,cfg,filename,figureTitle)
%PLOT_CORNER_ZOOM Plot fixed, correctly centred windows around every vertex.
if nargin<5 || isempty(filename), filename='02_four_corner_zooms.png'; end
if nargin<6 || isempty(figureTitle), figureTitle='Corner transition zooms'; end
zoomRadius=2.4; % [mm], slightly wider than the 2 mm corner transition
fig=figure('Visible',cfg.plot.visible,'Color','w','Position',[100 100 980 780]);
layout=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
colors=lines(numel(solutions)); legendHandles=gobjects(1,numel(solutions)+2);
for c=1:4
    ax=nexttile(layout); hold(ax,'on'); vertex=path.points(c,:);
    hTolerance=plot_tolerance_band(ax,path,cfg);
    hIdeal=plot(ax,path.points(:,1),path.points(:,2),'k--','LineWidth',1.2, ...
        'DisplayName','Ideal CL');
    hSolution=gobjects(1,numel(solutions));
    for i=1:numel(solutions)
        hSolution(i)=plot(ax,solutions{i}.position(1,:),solutions{i}.position(2,:), ...
            'LineWidth',1.25,'Color',colors(i,:),'DisplayName',labels{i});
    end
    % Set the data aspect first, then lock limits around the actual vertex.
    % The previous reverse order allowed axis equal to recenter the X limits.
    axis(ax,'equal');
    xlim(ax,vertex(1)+[-zoomRadius zoomRadius]);
    ylim(ax,vertex(2)+[-zoomRadius zoomRadius]);
    axis(ax,'manual'); grid(ax,'on');
    xlabel(ax,'X [mm]'); ylabel(ax,'Y [mm]');
    title(ax,sprintf('Corner %d: (%.0f, %.0f) mm',c,vertex(1),vertex(2)));
    if c==1, legendHandles=[hTolerance,hIdeal,hSolution]; end
end
title(layout,figureTitle);
lgd=legend(legendHandles,[{'Allowable tolerance band','Ideal CL'},labels], ...
    'Orientation','horizontal','Location','southoutside');
lgd.NumColumns=min(3,numel(legendHandles));
lgd.Layout.Tile='south';
save_publication_figure(fig,fullfile(cfg.output.figures,filename),cfg);
end
