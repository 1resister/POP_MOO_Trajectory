function plot_pareto_band_metrics(data,cfg)
%PLOT_PARETO_BAND_METRICS Show true resonance-band energy over the Pareto scan.
valid=logical(data.Feasible);
lambda=data.LambdaVibration(valid);
frequency=data.ResonanceFrequencyHz(find(valid,1,'first'));
fig=figure('Visible',cfg.plot.visible,'Color','w','Position',[100 100 900 620]);
layout=tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');

ax=nexttile(layout); hold(ax,'on');
plot(ax,lambda,data.BandEnergyX(valid),'-o','LineWidth',1.1);
plot(ax,lambda,data.BandEnergyY(valid),'-s','LineWidth',1.1);
plot(ax,lambda,data.BandEnergyTotal(valid),'-d','LineWidth',1.3);
grid(ax,'on'); xlabel(ax,'Vibration weight \lambda_{vib}');
ylabel(ax,'Band energy [-]');
legend(ax,'X axis','Y axis','Total','Location','best');
title(ax,sprintf('%g Hz \\pm bandwidth modal energy',frequency));

ax=nexttile(layout);
plot(ax,lambda,data.ReductionVsNoSuppression_percent(valid),'-o','LineWidth',1.2);
hold(ax,'on'); yline(ax,0,'k--','No-suppression reference'); grid(ax,'on');
xlabel(ax,'Vibration weight \lambda_{vib}'); ylabel(ax,'Reduction [%]');
title(ax,'Positive means resonance-band suppression');
title(layout,sprintf('%g Hz resonance-band result across the Pareto scan',frequency));
save_publication_figure(fig,fullfile(cfg.output.figures, ...
    '21_pareto_resonance_band_energy.png'),cfg);
end
