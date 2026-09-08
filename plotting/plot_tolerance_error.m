function plot_tolerance_error(solution,cfg)
c=solution.validation.contour;
fig=figure('Visible',cfg.plot.visible,'Color','w');
subplot(2,1,1); plot(solution.time,c.error,'LineWidth',1.1); hold on;
plot(solution.time,c.allowed_error,'--','LineWidth',1.1); grid on;
xlabel('Time [s]'); ylabel('Error [mm]'); title('Contour error and dynamic tolerance');
legend('Actual contour error','Allowed dynamic error','Location','best');
subplot(2,1,2); plot(solution.time,c.utilisation,'LineWidth',1.1); yline(1,'r--'); grid on;
xlabel('Time [s]'); ylabel('Error utilisation [-]'); title('Error utilisation ratio');
save_publication_figure(fig,fullfile(cfg.output.figures,'03_tolerance_error_utilisation.png'),cfg);

fig=figure('Visible',cfg.plot.visible,'Color','w');
plot(c.path_progress,c.allowed_error,'LineWidth',1.2); grid on;
xlabel('Ideal path progress [mm]'); ylabel('Allowed error [mm]');
title('Dynamic tolerance envelope along path');
save_publication_figure(fig,fullfile(cfg.output.figures,'04_tolerance_vs_path_progress.png'),cfg);
end
