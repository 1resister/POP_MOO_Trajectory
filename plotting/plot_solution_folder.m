function outputFolder = plot_solution_folder(path,solution,label,folderName,paretoData,cfg,highlightColor)
%PLOT_SOLUTION_FOLDER Export a complete, self-contained figure set for one solution.
if nargin<7 || isempty(highlightColor), highlightColor=[0.15 0.45 0.75]; end
outputFolder=fullfile(cfg.output.figures,folderName);
if ~exist(outputFolder,'dir'), mkdir(outputFolder); end

localCfg=cfg;
localCfg.output.figures=outputFolder;
plot_path_comparison(path,{solution},{label},localCfg);
plot_corner_zoom(path,{solution},{label},localCfg,[],sprintf('%s: corner transition zooms',label));
plot_tolerance_error(solution,localCfg);
plot_axis_kinematics(solution,localCfg);
plot_pop_states(solution,localCfg);
plot_straightness(solution,path,localCfg);
plot_single_modal_response(solution,label,localCfg);

frequency=calculate_frequency_metrics(solution,cfg);
plot_single_frequency_spectrum(frequency,label,localCfg);
plot_constraint_usage(solution,localCfg);
plot_single_pareto_context(paretoData,solution,label,highlightColor,localCfg);
write_solution_summary(solution,label,outputFolder);
end

function plot_single_modal_response(solution,label,cfg)
fig=figure('Visible',cfg.plot.visible,'Color','w');
axesNames={'X','Y'};
for axisIndex=1:2
    subplot(2,1,axisIndex);
    plot(solution.time,squeeze(solution.modal(axisIndex,1,:)),'LineWidth',1.1, ...
        'Color',[0.15 0.45 0.75]);
    grid on; xlabel('Time [s]'); ylabel(sprintf('q_%s [-]',lower(axesNames{axisIndex})));
    title(sprintf('%s: %s-axis 10 Hz modal response',label,axesNames{axisIndex}));
end
save_publication_figure(fig,fullfile(cfg.output.figures,'12_modal_response.png'),cfg);
end

function plot_single_frequency_spectrum(frequency,label,cfg)
plot_frequency_pair(frequency,{'ax','ay'},'Acceleration PSD', ...
    '(mm/s^2)^2/Hz','13_acceleration_psd.png',label,cfg);
plot_frequency_pair(frequency,{'modal_x','modal_y'},'Modal response PSD', ...
    '1/Hz','14_modal_response_psd.png',label,cfg);
end

function plot_frequency_pair(frequency,keys,titleText,unitText,filename,label,cfg)
fig=figure('Visible',cfg.plot.visible,'Color','w');
axesNames={'X','Y'};
for axisIndex=1:2
    subplot(2,1,axisIndex); spec=frequency.signal.(keys{axisIndex});
    plot(spec.f,10*log10(spec.psd+eps),'LineWidth',1.1, ...
        'Color',[0.15 0.45 0.75]);
    hold on; xline(10,'r--','10 Hz');
    xlim([0 cfg.frequency.maximum_plot_frequency]); grid on;
    xlabel('Frequency [Hz]'); ylabel(sprintf('PSD [%s, dB]',unitText));
    title(sprintf('%s: %s - %s axis',label,titleText,axesNames{axisIndex}));
end
save_publication_figure(fig,fullfile(cfg.output.figures,filename),cfg);
end

function plot_single_pareto_context(data,solution,label,highlightColor,cfg)
fig=figure('Visible',cfg.plot.visible,'Color','w'); valid=logical(data.Feasible);
hFront=scatter(data.Jstraightness(valid),data.Jvibration(valid),60, ...
    data.LambdaVibration(valid),'filled','DisplayName','Near-time Pareto scan');
hold on; grid on; cb=colorbar; cb.Label.String='Vibration weight \lambda_{vib}';
hSelected=plot(solution.objectives.straightness,solution.objectives.vibration, ...
    'p','MarkerSize',14,'MarkerFaceColor',highlightColor,'MarkerEdgeColor','k', ...
    'DisplayName',label);
xlabel('Straightness objective [-]'); ylabel('Vibration objective [-]');
title(sprintf('%s: position relative to the Pareto front',label));
legend([hFront,hSelected],{'Near-time Pareto scan',label},'Location','best');
save_publication_figure(fig,fullfile(cfg.output.figures,'15_pareto_context.png'),cfg);
end

function write_solution_summary(solution,label,outputFolder)
contourError=solution.validation.contour.error;
summary=table(string(label),solution.N,solution.T, ...
    solution.objectives.straightness,solution.objectives.vibration, ...
    max(contourError),sqrt(mean(contourError.^2)), ...
    'VariableNames',{'Solution','N','Time_s','Jstraightness','Jvibration', ...
    'MaxContourError_mm','ContourRMSE_mm'});
writetable(summary,fullfile(outputFolder,'solution_summary.csv'));
end
