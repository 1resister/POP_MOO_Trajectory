function outputFolder = plot_resonance_comparison_folder(path,noSuppression,suppression10Hz,cfg)
%PLOT_RESONANCE_COMPARISON_FOLDER Export resonance-suppression comparisons.
outputFolder=fullfile(cfg.output.figures,'resonance_comparison');
if ~exist(outputFolder,'dir'), mkdir(outputFolder); end
localCfg=cfg; localCfg.output.figures=outputFolder;
f0=suppression10Hz.model.mode(1).frequency;
frequencyLabel=sprintf('%g Hz',f0);
labels={'No resonance suppression',[frequencyLabel,' suppression (knee)']};

plot_path_comparison(path,{noSuppression,suppression10Hz},labels,localCfg);
plot_corner_zoom(path,{noSuppression,suppression10Hz},labels,localCfg,[], ...
    sprintf('No resonance suppression vs %s suppression',frequencyLabel));
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
plot_vibration_components(noSuppression,suppression10Hz,labels,localCfg);
plot_axis_rms_peak_metrics(noSuppression,suppression10Hz,labels,localCfg);
write_comparison_summary(noSuppression,suppression10Hz,frequencyNo,frequency10,labels,outputFolder,cfg);
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
frequencyLabel=sprintf('%g Hz',b.model.mode(1).frequency);
for axisIndex=1:2
    subplot(2,1,axisIndex); hold on;
    plot(a.time,squeeze(a.modal(axisIndex,1,:)),'LineWidth',1.0);
    plot(b.time,squeeze(b.modal(axisIndex,1,:)),'LineWidth',1.0); grid on;
    xlabel('Time [s]'); ylabel(sprintf('q_%s [-]',lower(axesNames{axisIndex})));
    title(sprintf('%s modal response - %s axis',frequencyLabel,axesNames{axisIndex}));
    legend(labels{1},labels{2},'Location','best');
end
save_publication_figure(fig,fullfile(cfg.output.figures,'13_modal_response.png'),cfg);
end

function plot_psd_comparison(fa,fb,keys,titleText,unitText,filename,labels,cfg)
fig=figure('Visible',cfg.plot.visible,'Color','w'); axesNames={'X','Y'};
f0=fb.mode(1).frequency;
frequencyLabel=sprintf('%g Hz',f0);
for axisIndex=1:2
    subplot(2,1,axisIndex); hold on;
    sa=fa.signal.(keys{axisIndex}); sb=fb.signal.(keys{axisIndex});
    plot(sa.f,10*log10(sa.psd+eps),'LineWidth',1.0);
    plot(sb.f,10*log10(sb.psd+eps),'LineWidth',1.0);
    xline(f0,'r--',frequencyLabel); xlim([0 cfg.frequency.maximum_plot_frequency]); grid on;
    xlabel('Frequency [Hz]'); ylabel(sprintf('PSD [%s, dB]',unitText));
    title(sprintf('%s - %s axis',titleText,axesNames{axisIndex}));
    legend(labels{1},labels{2},frequencyLabel,'Location','best');
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

function plot_metric_comparison(a,b,fa,fb,~,cfg)
bandA=modal_band_energy(fa); bandB=modal_band_energy(fb);
frequencyLabel=sprintf('%g Hz',fb.mode(1).frequency);
values=[a.T,b.T; max(a.validation.contour.error),max(b.validation.contour.error); ...
    sqrt(mean(a.validation.contour.error.^2)),sqrt(mean(b.validation.contour.error.^2)); ...
    a.objectives.straightness,b.objectives.straightness; ...
    a.objectives.vibration,b.objectives.vibration; bandA,bandB];
names={'Cycle time [s]','Maximum contour error [mm]','Contour RMSE [mm]', ...
    'Straightness objective','Vibration objective',[frequencyLabel,' modal band energy']};
fig=figure('Visible',cfg.plot.visible,'Color','w','Position',[100 100 980 620]);
layout=tiledlayout(fig,2,3,'TileSpacing','compact','Padding','compact');
for metricIndex=1:numel(names)
    ax=nexttile(layout); bar(ax,values(metricIndex,:)); grid(ax,'on');
    set(ax,'XTick',1:2,'XTickLabel',{'No suppression',frequencyLabel});
    title(ax,names{metricIndex});
end
title(layout,sprintf('No resonance suppression vs %s suppression: key metrics',frequencyLabel));
save_publication_figure(fig,fullfile(cfg.output.figures,'18_key_metrics.png'),cfg);
end

function plot_vibration_components(a,b,labels,cfg)
values=[weighted_vibration_components(a,cfg);weighted_vibration_components(b,cfg)].';
fig=figure('Visible',cfg.plot.visible,'Color','w');
bar(values); grid on;
set(gca,'XTick',1:7,'XTickLabel',{'Modal energy','Modal peak', ...
    'Acceleration RMS','Acceleration peak','Jerk RMS','Jerk peak', ...
    'Resonance band'});
ylabel('Weighted contribution to J_{vibration} [-]');
title('Vibration-objective components'); legend(labels{1},labels{2},'Location','best');
save_publication_figure(fig,fullfile(cfg.output.figures, ...
    '19_vibration_objective_components.png'),cfg);
end

function plot_axis_rms_peak_metrics(a,b,labels,cfg)
ca=a.objectives.vibration_components;
cb=b.objectives.vibration_components;
valuesA={ca.acceleration_rms_axis,ca.acceleration_peak_axis, ...
    ca.jerk_rms_axis,ca.jerk_peak_axis};
valuesB={cb.acceleration_rms_axis,cb.acceleration_peak_axis, ...
    cb.jerk_rms_axis,cb.jerk_peak_axis};
titles={'Acceleration RMS / A_{max}','Acceleration peak / A_{max}', ...
    'Jerk RMS / J_{max}','Jerk peak / J_{max}'};
fig=figure('Visible',cfg.plot.visible,'Color','w','Position',[100 100 920 620]);
layout=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
for metricIndex=1:4
    ax=nexttile(layout);
    bar(ax,[valuesA{metricIndex}(:),valuesB{metricIndex}(:)]);
    grid(ax,'on');
    set(ax,'XTick',1:2,'XTickLabel',{'X','Y'});
    ylabel(ax,'Normalized metric [-]');
    title(ax,titles{metricIndex});
end
legend(nexttile(layout,1),labels{1},labels{2},'Location','best');
title(layout,'X/Y acceleration and jerk metrics included in J_{vibration}');
save_publication_figure(fig,fullfile(cfg.output.figures, ...
    '20_axis_rms_peak_metrics.png'),cfg);
end

function write_comparison_summary(a,b,fa,fb,labels,outputFolder,cfg)
bandA=modal_band_energy(fa); bandB=modal_band_energy(fb);
componentsA=weighted_vibration_components(a,cfg);
componentsB=weighted_vibration_components(b,cfg);
rawA=a.objectives.vibration_components;
rawB=b.objectives.vibration_components;
summary=table(string(labels(:)),[a.N;b.N],[a.T;b.T], ...
    [a.objectives.straightness;b.objectives.straightness], ...
    [a.objectives.vibration;b.objectives.vibration], ...
    [componentsA(1);componentsB(1)],[componentsA(2);componentsB(2)], ...
    [componentsA(3);componentsB(3)],[componentsA(4);componentsB(4)], ...
    [componentsA(5);componentsB(5)],[componentsA(6);componentsB(6)], ...
    [componentsA(7);componentsB(7)], ...
    [rawA.acceleration_rms_axis(1);rawB.acceleration_rms_axis(1)], ...
    [rawA.acceleration_rms_axis(2);rawB.acceleration_rms_axis(2)], ...
    [rawA.acceleration_peak_axis(1);rawB.acceleration_peak_axis(1)], ...
    [rawA.acceleration_peak_axis(2);rawB.acceleration_peak_axis(2)], ...
    [rawA.jerk_rms_axis(1);rawB.jerk_rms_axis(1)], ...
    [rawA.jerk_rms_axis(2);rawB.jerk_rms_axis(2)], ...
    [rawA.jerk_peak_axis(1);rawB.jerk_peak_axis(1)], ...
    [rawA.jerk_peak_axis(2);rawB.jerk_peak_axis(2)], ...
    [max(a.validation.contour.error);max(b.validation.contour.error)], ...
    [sqrt(mean(a.validation.contour.error.^2));sqrt(mean(b.validation.contour.error.^2))], ...
    [bandA;bandB], ...
    'VariableNames',{'Solution','N','Time_s','Jstraightness','Jvibration', ...
    'JmodalEnergy','JmodalPeak','JaccelerationRMS','JaccelerationPeak', ...
    'JjerkRMS','JjerkPeak','JresonanceBand', ...
    'AccelerationRMSX','AccelerationRMSY','AccelerationPeakX','AccelerationPeakY', ...
    'JerkRMSX','JerkRMSY','JerkPeakX','JerkPeakY', ...
    'MaxContourError_mm','ContourRMSE_mm','ResonanceBandEnergy'});
writetable(summary,fullfile(outputFolder,'comparison_summary.csv'));
reduction=table(100*(bandA-bandB)/max(bandA,eps), ...
    100*(a.objectives.vibration-b.objectives.vibration)/max(a.objectives.vibration,eps), ...
    100*(sum(componentsA(3:4))-sum(componentsB(3:4)))/max(sum(componentsA(3:4)),eps), ...
    100*(sum(componentsA(5:6))-sum(componentsB(5:6)))/max(sum(componentsA(5:6)),eps), ...
    'VariableNames',{'ModalBandEnergyReduction_percent', ...
    'VibrationObjectiveReduction_percent','AccelerationContributionReduction_percent', ...
    'JerkContributionReduction_percent'});
writetable(reduction,fullfile(outputFolder,'suppression_reduction.csv'));
end

function energy=modal_band_energy(frequency)
energy=frequency.mode(1).band_energy.modal_x+frequency.mode(1).band_energy.modal_y;
end

function values=weighted_vibration_components(solution,cfg)
components=solution.objectives.vibration_components;
values=[cfg.objective.vibration_energy_weight*components.energy, ...
    cfg.objective.vibration_peak_weight*components.peak_squared, ...
    cfg.objective.acceleration_rms_weight*components.acceleration_rms, ...
    cfg.objective.acceleration_peak_weight*components.acceleration_peak, ...
    cfg.objective.jerk_rms_weight*components.jerk_rms, ...
    cfg.objective.jerk_peak_weight*components.jerk_peak, ...
    components.resonance_band_contribution];
end
