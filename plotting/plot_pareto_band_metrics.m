function plot_pareto_band_metrics(data,cfg)
%PLOT_PARETO_BAND_METRICS Target-band result along the time Pareto front.
valid=data.Feasible;
frequency=data.ResonanceFrequencyHz(find(valid,1,'first'));
fig=figure('Visible',cfg.plot.visible,'Color','w','Position',[100 100 900 650]);
layout=tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');

ax=nexttile(layout); hold(ax,'on');
semilogy(ax,data.Time_s(valid),data.PairedNoSuppressionBandEnergy(valid), ...
    '--o','Color',[0.55 0.55 0.55],'LineWidth',1.0);
semilogy(ax,data.Time_s(valid),data.BandEnergyTotal(valid), ...
    '-s','Color',[0.10 0.35 0.75],'LineWidth',1.2);
grid(ax,'on'); xlabel(ax,'Machining time T [s]');
ylabel(ax,'Modal PSD band energy');
legend(ax,'No-suppression reference','Resonance suppression','Location','best');
title(ax,sprintf('%g Hz band energy for every time-vibration candidate',frequency));

ax=nexttile(layout); hold(ax,'on');
plot(ax,data.Time_s(valid),data.ReductionVsPairedNoSuppression_percent(valid), ...
    '-o','Color',[0.15 0.60 0.30],'LineWidth',1.2);
yline(ax,100*cfg.objective.resonance_band_min_reduction, ...
    'r--','Required minimum');
grid(ax,'on'); xlabel(ax,'Machining time T [s]'); ylabel(ax,'Reduction [%]');
title(ax,'Positive values mean target-band suppression');
title(layout,sprintf('%g Hz suppression along the time-vibration Pareto front', ...
    frequency));
save_publication_figure(fig,fullfile(cfg.output.figures, ...
    '21_pareto_resonance_band_energy.png'),cfg);
end
