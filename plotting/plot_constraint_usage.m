function plot_constraint_usage(solution,cfg)
u=solution.validation.usage;
fig=figure('Visible',cfg.plot.visible,'Color','w');
bar(100*u.ratio); yline(100,'r--'); grid on; ylim([0,max(110,105*max(u.ratio,[],'all'))]);
set(gca,'XTick',1:6,'XTickLabel',u.names); ylabel('Constraint utilisation [%]');
title('Independent hard-constraint utilisation'); legend('X axis','Y axis','Limit','Location','best');
save_publication_figure(fig,fullfile(cfg.output.figures,'16_constraint_utilisation.png'),cfg);
end
