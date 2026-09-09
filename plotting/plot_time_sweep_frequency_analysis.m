function rootFolder = plot_time_sweep_frequency_analysis(pairs,data,cfg)
%PLOT_TIME_SWEEP_FREQUENCY_ANALYSIS Export PSD and FFT for every time pair.
rootFolder=fullfile(cfg.output.figures,'pareto_time_sweep');
if ~exist(rootFolder,'dir'), mkdir(rootFolder); end
if isempty(pairs), return; end

for index=1:numel(pairs)
    pair=pairs{index};
    folder=fullfile(rootFolder,time_folder_tag(pair.N,pair.T));
    if ~exist(folder,'dir'), mkdir(folder); end
    localCfg=cfg;
    localCfg.output.figures=folder;
    frequencyNo=calculate_frequency_metrics(pair.no_suppression,cfg);
    frequencySupp=calculate_frequency_metrics(pair.suppression,cfg);
    spectrumNo=calculate_amplitude_spectrum(pair.no_suppression,cfg);
    spectrumSupp=calculate_amplitude_spectrum(pair.suppression,cfg);
    plot_all_psd(frequencyNo,frequencySupp,pair.T,localCfg);
    plot_all_fft(spectrumNo,spectrumSupp,pair.suppression.model.mode(1).frequency, ...
        pair.T,localCfg);
    save_trajectory_csv(pair.no_suppression, ...
        fullfile(folder,'trajectory_no_suppression.csv'));
    save_trajectory_csv(pair.suppression, ...
        fullfile(folder,'trajectory_resonance_suppression.csv'));
    write_pair_summary(pair,fullfile(folder,'comparison_summary.csv'));
    write_frequency_summary(frequencyNo,frequencySupp,spectrumNo,spectrumSupp, ...
        pair.suppression.model.mode(1).frequency, ...
        fullfile(folder,'frequency_summary.csv'));
end

plot_sweep_summary(data,cfg,rootFolder);
end

function plot_all_psd(noSuppression,suppression,T,cfg)
groups={{'ax','ay','Acceleration','(mm/s^2)^2/Hz'}, ...
    {'jx','jy','Jerk','(mm/s^3)^2/Hz'}, ...
    {'modal_x','modal_y','Modal response','1/Hz'}};
f0=suppression.mode(1).frequency;
bandNo=noSuppression.mode(1).band_energy.modal_x+ ...
    noSuppression.mode(1).band_energy.modal_y;
bandSupp=suppression.mode(1).band_energy.modal_x+ ...
    suppression.mode(1).band_energy.modal_y;
bandReduction=100*(bandNo-bandSupp)/max(bandNo,eps);
fig=figure('Visible',cfg.plot.visible,'Color','w','Position',[80 60 1200 900]);
layout=tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
for row=1:3
    group=groups{row};
    for axisIndex=1:2
        ax=nexttile(layout); hold(ax,'on');
        a=noSuppression.signal.(group{axisIndex});
        b=suppression.signal.(group{axisIndex});
        noLine=plot(ax,a.f,10*log10(a.psd+eps),'LineWidth',1.0);
        suppressionLine=plot(ax,b.f,10*log10(b.psd+eps),'LineWidth',1.0);
        targetLine=add_frequency_markers(ax,f0,cfg.frequency.bandwidth);
        xlim(ax,[0 cfg.frequency.maximum_plot_frequency]); grid(ax,'on');
        xlabel(ax,'Frequency [Hz]');
        ylabel(ax,sprintf('PSD [%s, dB]',group{4}));
        title(ax,sprintf('%s PSD - %s axis',group{3},char('X'+axisIndex-1)));
        if row==1 && axisIndex==1
            legend(ax,[noLine,suppressionLine,targetLine], ...
                'No suppression',sprintf('%g Hz suppression',f0), ...
                'Target frequency','Location','best');
        end
    end
end
title(layout,sprintf(['PSD comparison at machining time T = %.3f s; ' ...
    '%g Hz band reduction = %.2f%%'],T,f0,bandReduction));
save_publication_figure(fig,fullfile(cfg.output.figures,'01_psd_comparison.png'),cfg);
end

function plot_all_fft(noSuppression,suppression,f0,T,cfg)
groups={{'ax','ay','Acceleration','mm/s^2'}, ...
    {'jx','jy','Jerk','mm/s^3'}, ...
    {'modal_x','modal_y','Modal response','-'}};
fig=figure('Visible',cfg.plot.visible,'Color','w','Position',[80 60 1200 900]);
layout=tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
for row=1:3
    group=groups{row};
    for axisIndex=1:2
        ax=nexttile(layout); hold(ax,'on');
        a=noSuppression.signal.(group{axisIndex});
        b=suppression.signal.(group{axisIndex});
        noLine=plot(ax,a.f,20*log10(a.amplitude+eps),'LineWidth',1.0);
        suppressionLine=plot(ax,b.f,20*log10(b.amplitude+eps),'LineWidth',1.0);
        targetLine=add_frequency_markers(ax,f0,cfg.frequency.bandwidth);
        xlim(ax,[0 cfg.frequency.maximum_plot_frequency]); grid(ax,'on');
        xlabel(ax,'Frequency [Hz]');
        ylabel(ax,sprintf('Amplitude [%s, dB]',group{4}));
        title(ax,sprintf('%s FFT spectrum - %s axis',group{3},char('X'+axisIndex-1)));
        if row==1 && axisIndex==1
            legend(ax,[noLine,suppressionLine,targetLine], ...
                'No suppression',sprintf('%g Hz suppression',f0), ...
                'Target frequency','Location','best');
        end
    end
end
title(layout,sprintf('FFT amplitude-spectrum comparison at machining time T = %.3f s',T));
save_publication_figure(fig,fullfile(cfg.output.figures,'02_fft_spectrum_comparison.png'),cfg);
end

function targetLine=add_frequency_markers(ax,f0,bandwidth)
targetLine=xline(ax,f0,'r--',sprintf('%g Hz',f0),'LineWidth',1.0);
xline(ax,f0-bandwidth,'k:','LineWidth',0.8,'HandleVisibility','off');
xline(ax,f0+bandwidth,'k:','LineWidth',0.8,'HandleVisibility','off');
end

function plot_sweep_summary(data,cfg,rootFolder)
fig=figure('Visible',cfg.plot.visible,'Color','w','Position',[100 100 920 650]);
layout=tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');
ax=nexttile(layout); hold(ax,'on');
semilogy(ax,data.Time_s,data.NoSuppressionBandEnergy,'-o','LineWidth',1.1);
semilogy(ax,data.Time_s,data.SuppressedBandEnergy,'-s','LineWidth',1.1);
grid(ax,'on'); xlabel(ax,'Machining time [s]'); ylabel(ax,'Modal PSD band energy');
legend(ax,'No suppression','Resonance suppression','Location','best');
title(ax,sprintf('%g Hz band energy at every integer machining time', ...
    cfg.resonance.mode(1).frequency));
ax=nexttile(layout); hold(ax,'on');
plot(ax,data.Time_s,data.BandReductionPercent,'-o','LineWidth',1.1);
yline(ax,100*cfg.objective.resonance_band_min_reduction,'r--','Required minimum');
grid(ax,'on'); xlabel(ax,'Machining time [s]'); ylabel(ax,'Band reduction [%]');
title(ax,'Positive values mean PSD-band suppression');
title(layout,'Full machining-time vibration-optimization sweep');
save_publication_figure(fig,fullfile(rootFolder,'00_pareto_time_sweep_summary.png'),cfg);
end

function write_pair_summary(pair,filename)
no=pair.no_suppression;
supp=pair.suppression;
noBand=pair.band_reference.actual;
suppBand=supp.validation.resonance_band.actual_total;
summary=table(["No suppression";"Resonance suppression"], ...
    [no.N;supp.N],[no.T;supp.T], ...
    [no.objectives.vibration;supp.objectives.vibration], ...
    [noBand;suppBand], ...
    [0;100*(noBand-suppBand)/noBand], ...
    [max(no.validation.contour.error);max(supp.validation.contour.error)], ...
    [no.validation.pass;supp.validation.pass], ...
    'VariableNames',{'Solution','N','Time_s','Jvibration', ...
    'ResonanceBandEnergy','BandReductionPercent', ...
    'MaxContourError_mm','ValidationPass'});
writetable(summary,filename);
end

function write_frequency_summary(frequencyNo,frequencySupp,spectrumNo, ...
    spectrumSupp,f0,filename)
keys={'ax','ay','jx','jy','modal_x','modal_y'};
labels=["Acceleration X";"Acceleration Y";"Jerk X";"Jerk Y"; ...
    "Modal response X";"Modal response Y"];
n=numel(keys);
psdNo=zeros(n,1); psdSupp=zeros(n,1);
fftNo=zeros(n,1); fftSupp=zeros(n,1);
for index=1:n
    key=keys{index};
    psdNo(index)=value_at_frequency(frequencyNo.signal.(key).f, ...
        frequencyNo.signal.(key).psd,f0);
    psdSupp(index)=value_at_frequency(frequencySupp.signal.(key).f, ...
        frequencySupp.signal.(key).psd,f0);
    fftNo(index)=value_at_frequency(spectrumNo.signal.(key).f, ...
        spectrumNo.signal.(key).amplitude,f0);
    fftSupp(index)=value_at_frequency(spectrumSupp.signal.(key).f, ...
        spectrumSupp.signal.(key).amplitude,f0);
end
summary=table(labels,repmat(f0,n,1),psdNo,psdSupp, ...
    100*(psdNo-psdSupp)./max(psdNo,eps),fftNo,fftSupp, ...
    100*(fftNo-fftSupp)./max(fftNo,eps), ...
    'VariableNames',{'Signal','TargetFrequency_Hz','NoSuppressionPSD', ...
    'SuppressedPSD','PSDReductionPercent','NoSuppressionFFTAmplitude', ...
    'SuppressedFFTAmplitude','FFTAmplitudeReductionPercent'});
writetable(summary,filename);
end

function value=value_at_frequency(frequency,data,target)
[~,index]=min(abs(frequency-target));
value=data(index);
end

function tag=time_folder_tag(N,T)
timeText=strrep(sprintf('%.3f',T),'.','p');
tag=sprintf('N_%04d_T_%ss',N,timeText);
end
