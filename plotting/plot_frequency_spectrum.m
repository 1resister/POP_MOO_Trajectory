function plot_frequency_spectrum(fA,fB,cfg)
plot_pair(fA,fB,{'ax','ay'},'Acceleration PSD',{'(mm/s^2)^2/Hz','(mm/s^2)^2/Hz'}, ...
    '13_acceleration_psd.png',cfg);
plot_pair(fA,fB,{'modal_x','modal_y'},'Modal response PSD',{'1/Hz','1/Hz'}, ...
    '14_modal_response_psd.png',cfg);
fig=figure('Visible',cfg.plot.visible,'Color','w'); hold on;
a=fA.signal.modal_x; b=fB.signal.modal_x;
plot(a.f,10*log10(a.psd+eps),'LineWidth',1); plot(b.f,10*log10(b.psd+eps),'LineWidth',1);
xline(10,'r--','10 Hz'); xlim([0 cfg.frequency.maximum_plot_frequency]); grid on;
xlabel('Frequency [Hz]'); ylabel('Modal PSD [dB/Hz]');
title('No suppression vs 10 Hz suppression'); legend('No suppression','10 Hz suppression');
save_publication_figure(fig,fullfile(cfg.output.figures,'15_suppression_spectrum_comparison.png'),cfg);
end

function plot_pair(fA,fB,keys,titleText,units,file,cfg)
fig=figure('Visible',cfg.plot.visible,'Color','w');
for axis=1:2
    subplot(2,1,axis); hold on; a=fA.signal.(keys{axis}); b=fB.signal.(keys{axis});
    plot(a.f,10*log10(a.psd+eps),'LineWidth',1); plot(b.f,10*log10(b.psd+eps),'LineWidth',1);
    xline(10,'r--','10 Hz'); xlim([0 cfg.frequency.maximum_plot_frequency]); grid on;
    xlabel('Frequency [Hz]'); ylabel(sprintf('PSD [%s, dB]',units{axis}));
    title(sprintf('%s - %s axis',titleText,char('X'+axis-1))); legend('No suppression','10 Hz suppression');
end
save_publication_figure(fig,fullfile(cfg.output.figures,file),cfg);
end
