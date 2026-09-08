function save_publication_figure(fig,filename,cfg)
%SAVE_PUBLICATION_FIGURE Apply consistent paper-style export settings.
set(findall(fig,'-property','FontName'),'FontName','Times New Roman');
set(findall(fig,'-property','FontSize'),'FontSize',10);
set(findall(fig,'Type','axes'),'LineWidth',0.8,'Box','on');
exportgraphics(fig,filename,'Resolution',cfg.plot.resolution);
close(fig);
end
