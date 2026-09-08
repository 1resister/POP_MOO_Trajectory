function plot_resonance_response(sA,sB,cfg)
fig=figure('Visible',cfg.plot.visible,'Color','w');
subplot(2,1,1); plot(sA.time,squeeze(sA.modal(1,1,:)),'LineWidth',1); hold on;
plot(sB.time,squeeze(sB.modal(1,1,:)),'LineWidth',1); grid on;
xlabel('Time [s]'); ylabel('q_x [-]'); title('X-axis 10 Hz modal response'); legend('No suppression','10 Hz suppression');
subplot(2,1,2); plot(sA.time,squeeze(sA.modal(2,1,:)),'LineWidth',1); hold on;
plot(sB.time,squeeze(sB.modal(2,1,:)),'LineWidth',1); grid on;
xlabel('Time [s]'); ylabel('q_y [-]'); title('Y-axis 10 Hz modal response');
save_publication_figure(fig,fullfile(cfg.output.figures,'12_modal_response.png'),cfg);
end
