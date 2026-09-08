function outputFolder = plot_resonance_comparison_folder(path,noSuppression,suppression10Hz,cfg)
%PLOT_RESONANCE_COMPARISON_FOLDER Export all no-suppression vs 10 Hz comparisons.
outputFolder=fullfile(cfg.output.figures,'resonance_comparison');
if ~exist(outputFolder,'dir'), mkdir(outputFolder); end
localCfg=cfg; localCfg.output.figures=outputFolder;
labels={'No resonance suppression','10 Hz suppression (knee)'};

plot_path_comparison(path,{noSuppression,suppression10Hz},labels,localCfg);
plot_corner_zoom(path,{noSuppression,suppression10Hz},labels,localCfg,[], ...
    'No resonance suppression vs 10 Hz suppression');
plot_contour_comparison(noSuppression,suppression10Hz,labels,localCfg);
plot_signal_comparison(noSuppression,suppression10Hz,'position','Position','mm', ...
    '05_position.png',[],labels,localCfg);
plot_signal_comparison(noSuppression,suppression10Hz,'velocity','Velocity','mm/s', ...
    '06_velocity.png',cfg.limits.Vmax,labels,localCfg);
plot_signal_comparison(noSuppression,suppression10Hz,'acceleration','Acceleration','mm/s^2', ...
    '07_acceleration.png',cfg.limits.Amax,labels,localCfg);
plot_signal_comparison(noSuppression,suppression10Hz,'jerk','Jerk','mm/s^3', ...
    '08_jerk.png',cfg.limits.Jmax,labels,localCfg);
plot_signal_comparison(noSuppression,suppression10Hz,'snap','Snap','mm/s^4', ...
    '09_snap.png',cfg.limits.SnapMax,labels,localCfg);
plot_signal_comparison(noSuppression,suppression10Hz,'crackle','Crackle','mm/s^5', ...
    '10_crackle.png',cfg.limits.CrackleMax,labels,localCfg);
plot_pop_comparison(noSuppression,suppression10Hz,labels,localCfg);
plot_transverse_comparison(noSuppression,suppression10Hz,path,labels,localCfg);
plot_modal_comparison(noSuppression,suppression10Hz,labels,localCfg);

frequencyNo=calculate_frequency_metrics(noSuppression,cfg);
frequency10=calculate_frequency_metrics(suppression10Hz,cfg);
plot_psd_comparison(frequencyNo,frequency10,{'ax','ay'},'Acceleration PSD', ...
    '(mm/s^2)^2/Hz','14_acceleration_psd.png',labels,localCfg);
plot_psd_comparison(frequencyNo,frequency10,{'jx','jy'},'Jerk PSD', ...
    '(mm/s^3)^2/Hz','15_jerk_psd.png',labels,localCfg);
plot_psd_comparison(frequencyNo,frequency10,{'modal_x','modal_y'},'Modal response PSD', ...
    '1/Hz','16_modal_response_psd.png',labels,localCfg);
plot_usage_comparison(noSuppression,suppression10Hz,labels,localCfg);
plot_metric_comparison(noSuppression,suppression10Hz,frequencyNo,frequency10,labels,localCfg);
write_comparison_summary(noSuppression,suppression10Hz,frequencyNo,frequency10,labels,outputFolder);
end

function plot_contour_comparison(a,b,labels,cfg)
ca=a.validation.contour; cb=b.validation.contour;
fig=figure('Visible',cfg.plot.visible,'Color','w');
subplot(2,1,1); hold on;
plot(a.time,ca.error,'LineWidth',1.1); plot(b.time,cb.error,'LineWidth',1.1);
plot(a.time,ca.allowed_error,'--','LineWidth',1.0); plot(b.time,cb.allowed_error,'--','LineWidth',1.0);
grid on; xlabel('Time [s]'); ylabel('Error [mm]'); title('Contour error and dynamic tolerance');
legend(labels{1},labels{2},[labels{1},' tolerance'],[labels{2},' tolerance'],'Location','best');
subplot(2,1,2); hold on;
plot(a.time,ca.utilisation,'LineWidth',1.1); plot(b.time,cb.utilisation,'LineWidth',1.1);
yline(1,'r--','Limit'); grid on; xlabel('Time [s]'); ylabel('Error utilisation [-]');
title('Dynamic-tolerance utilisation'); legend(labels{1},labels{2},'Limit','Location','best');
save_publication_figure(fig,fullfile(cfg.output.figures,'03_contour_error_comparison.png'),cfg);

fig=figure('Visible',cfg.plot.visible,'Color','w'); hold on;
plot(ca.path_progress,ca.error,'LineWidth',1.1); plot(cb.path_progress,cb.error,'LineWidth',1.1);
plot(ca.path_progress,ca.allowed_error,'--','LineWidth',1.0); plot(cb.path_progress,cb.allowed_error,'--','LineWidth',1.0);
grid on; xlabel('Ideal path progress [mm]'); ylabel('Error [mm]');
title('Contour error along the ideal path');
legend(labels{1},labels{2},[labels{1},' tolerance'],[labels{2},' tolerance'],'Location','best');
save_publication_figure(fig,fullfile(cfg.output.figures,'04_contour_error_along_path.png'),cfg);
end

function plot_signal_comparison(a,b,fieldName,titleText,unitText,filename,limit,labels,cfg)
signalA=a.(fieldName); signalB=b.(fieldName); axesNames={'X','Y'};
fig=figure('Visible',cfg.plot.visible,'Color','w');
for axisIndex=1:2
    subplot(2,1,axisIndex); hold on;
    plot(a.time,signalA(axisIndex,:),'LineWidth',1.0);
    plot(b.time,signalB(axisIndex,:),'LineWidth',1.0);
    if ~isempty(limit), yline(limit,'k--'); yline(-limit,'k--'); end
    grid on; xlabel('Time [s]'); ylabel(sprintf('%s_%s [%s]',lower(titleText),lower(axesNames{axisIndex}),unitText));
    title(sprintf('%s - %s axis',titleText,axesNames{axisIndex}));
    if isempty(limit)
        legend(labels{1},labels{2},'Location','best');
    else
        legend(labels{1},labels{2},'Limits','Location','best');
    end
end
save_publication_figure(fig,fullfile(cfg.output.figures,filename),cfg);
end

function plot_pop_comparison(a,b,labels,cfg)
popA=[a.U,a.U(:,end)]; popB=[b.U,b.U(:,end)]; axesNames={'X','Y'};
fig=figure('Visible',cfg.plot.visible,'Color','w');
for axisIndex=1:2
    subplot(2,1,axisIndex); hold on;
    plot(a.time,popA(axisIndex,:),'LineWidth',1.0); plot(b.time,popB(axisIndex,:),'LineWidth',1.0);
    yline(cfg.limits.POPMax,'k--'); yline(-cfg.limits.POPMax,'k--'); grid on;
    xlabel('Time [s]'); ylabel(sprintf('POP_%s [mm/s^6]',lower(axesNames{axisIndex})));
    title(sprintf('POP - %s axis',axesNames{axisIndex}));
    legend(labels{1},labels{2},'Limits','Location','best');
end
save_publication_figure(fig,fullfile(cfg.output.figures,'11_pop.png'),cfg);
end

function plot_transverse_comparison(a,b,path,labels,cfg)
va=transverse_velocity(a,path); vb=transverse_velocity(b,path);
fig=figure('Visible',cfg.plot.visible,'Color','w'); hold on;
plot(a.time,va,'LineWidth',1.0); plot(b.time,vb,'LineWidth',1.0); yline(0,'k:'); grid on;
xlabel('Time [s]'); ylabel('Transverse velocity v_{perp} [mm/s]');
title('Straight-line transverse velocity'); legend(labels{1},labels{2},'Location','best');
save_publication_figure(fig,fullfile(cfg.output.figures,'12_transverse_velocity.png'),cfg);
end

function value=transverse_velocity(solution,path)
value=nan(1,solution.N+1); cursor=0;
for segmentIndex=1:path.n_segments
    idx=cursor+(1:solution.Nvec(segmentIndex)+1);
    normal=path.segment(segmentIndex).normal(1:2);
    value(idx)=normal*solution.velocity(:,idx);
    cursor=cursor+solution.Nvec(segmentIndex);
end
end

function plot_modal_comparison(a,b,labels,cfg)
fig=figure('Visible',cfg.plot.visible,'Color','w'); axesNames={'X','Y'};
for axisIndex=1:2
    subplot(2,1,axisIndex); hold on;
    plot(a.time,squeeze(a.modal(axisIndex,1,:)),'LineWidth',1.0);
    plot(b.time,squeeze(b.modal(axisIndex,1,:)),'LineWidth',1.0); grid on;
    xlabel('Time [s]'); ylabel(sprintf('q_%s [-]',lower(axesNames{axisIndex})));
    title(sprintf('10 Hz modal response - %s axis',axesNames{axisIndex}));
    legend(labels{1},labels{2},'Location','best');
end
save_publication_figure(fig,fullfile(cfg.output.figures,'13_modal_response.png'),cfg);
end

function plot_psd_comparison(fa,fb,keys,titleText,unitText,filename,labels,cfg)
fig=figure('Visible',cfg.plot.visible,'Color','w'); axesNames={'X','Y'};
for axisIndex=1:2
    subplot(2,1,axisIndex); hold on;
    sa=fa.signal.(keys{axisIndex}); sb=fb.signal.(keys{axisIndex});
    plot(sa.f,10*log10(sa.psd+eps),'LineWidth',1.0);
    plot(sb.f,10*log10(sb.psd+eps),'LineWidth',1.0);
    xline(10,'r--','10 Hz'); xlim([0 cfg.frequency.maximum_plot_frequency]); grid on;
    xlabel('Frequency [Hz]'); ylabel(sprintf('PSD [%s, dB]',unitText));
    title(sprintf('%s - %s axis',titleText,axesNames{axisIndex}));
    legend(labels{1},labels{2},'10 Hz','Location','best');
end
save_publication_figure(fig,fullfile(cfg.output.figures,filename),cfg);
end

function plot_usage_comparison(a,b,labels,cfg)
ua=100*a.validation.usage.ratio; ub=100*b.validation.usage.ratio;
fig=figure('Visible',cfg.plot.visible,'Color','w');
bar([ua,ub]); hold on; yline(100,'r--','Limit'); grid on;
set(gca,'XTick',1:6,'XTickLabel',a.validation.usage.names);
ylabel('Constraint utilisation [%]'); title('Independent hard-constraint utilisation');
legend([labels{1},' X'],[labels{1},' Y'],[labels{2},' X'],[labels{2},' Y'],'Location','best');
save_publication_figure(fig,fullfile(cfg.output.figures,'17_constraint_utilisation.png'),cfg);
end

function plot_metric_comparison(a,b,fa,fb,labels,cfg)
bandA=modal_band_energy(fa); bandB=modal_band_energy(fb);
values=[a.T,b.T; max(a.validation.contour.error),max(b.validation.contour.error); ...
    sqrt(mean(a.validation.contour.error.^2)),sqrt(mean(b.validation.contour.error.^2)); ...
    a.objectives.straightness,b.objectives.straightness; ...
    a.objectives.vibration,b.objectives.vibration; bandA,bandB];
names={'Cycle time [s]','Maximum contour error [mm]','Contour RMSE [mm]', ...
    'Straightness objective','Vibration objective','10 Hz modal band energy'};
fig=figure('Visible',cfg.plot.visible,'Color','w','Position',[100 100 980 620]);
layout=tiledlayout(fig,2,3,'TileSpacing','compact','Padding','compact');
for metricIndex=1:numel(names)
    ax=nexttile(layout); bar(ax,values(metricIndex,:)); grid(ax,'on');
    set(ax,'XTick',1:2,'XTickLabel',{'No suppression','10 Hz'});
    title(ax,names{metricIndex});
end
title(layout,'No resonance suppression vs 10 Hz suppression: key metrics');
save_publication_figure(fig,fullfile(cfg.output.figures,'18_key_metrics.png'),cfg);
end

function write_comparison_summary(a,b,fa,fb,labels,outputFolder)
bandA=modal_band_energy(fa); bandB=modal_band_energy(fb);
summary=table(string(labels(:)),[a.N;b.N],[a.T;b.T], ...
    [a.objectives.straightness;b.objectives.straightness], ...
    [a.objectives.vibration;b.objectives.vibration], ...
    [max(a.validation.contour.error);max(b.validation.contour.error)], ...
    [sqrt(mean(a.validation.contour.error.^2));sqrt(mean(b.validation.contour.error.^2))], ...
    [bandA;bandB], ...
    'VariableNames',{'Solution','N','Time_s','Jstraightness','Jvibration', ...
    'MaxContourError_mm','ContourRMSE_mm','ModalBandEnergy10Hz'});
writetable(summary,fullfile(outputFolder,'comparison_summary.csv'));
reduction=table(100*(bandA-bandB)/max(bandA,eps), ...
    100*(a.objectives.vibration-b.objectives.vibration)/max(a.objectives.vibration,eps), ...
    'VariableNames',{'ModalBandEnergyReduction_percent','VibrationObjectiveReduction_percent'});
writetable(reduction,fullfile(outputFolder,'suppression_reduction.csv'));
end

function energy=modal_band_energy(frequency)
energy=frequency.mode(1).band_energy.modal_x+frequency.mode(1).band_energy.modal_y;
end
