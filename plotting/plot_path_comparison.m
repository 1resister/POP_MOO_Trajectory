function plot_path_comparison(path,solutions,labels,cfg)
fig=figure('Visible',cfg.plot.visible,'Color','w'); ax=axes(fig); hold(ax,'on');
hTolerance=plot_tolerance_band(ax,path,cfg);
hIdeal=plot(ax,path.points(:,1),path.points(:,2),'k--','LineWidth',1.4,'DisplayName','Ideal CL');
colors=lines(numel(solutions)); hSolutions=gobjects(1,numel(solutions));
for i=1:numel(solutions)
    hSolutions(i)=plot(ax,solutions{i}.position(1,:),solutions{i}.position(2,:), ...
        'LineWidth',1.1,'Color',colors(i,:),'DisplayName',labels{i});
end
grid(ax,'on'); xlabel(ax,'X [mm]'); ylabel(ax,'Y [mm]');
padding=max(0.5,3*max(cfg.geometry.line_tolerance,cfg.geometry.corner_tolerance));
axis(ax,'equal');
xlim(ax,[min(path.points(:,1))-padding,max(path.points(:,1))+padding]);
ylim(ax,[min(path.points(:,2))-padding,max(path.points(:,2))+padding]);
axis(ax,'manual');
title(ax,'Ideal path, tolerance band and optimised toolpaths');
legend([hTolerance,hIdeal,hSolutions],[{'Allowable tolerance band','Ideal CL'},labels], ...
    'Location','best');
save_publication_figure(fig,fullfile(cfg.output.figures,'01_toolpath_comparison.png'),cfg);
end
