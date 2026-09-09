function artifacts = finalize_outputs(cfg,path,solution_min_time, ...
    solution_no_resonance,solution_10Hz,pareto_data,pareto_solutions, ...
    test_results,stage_a_search,stage_b_info,time_sweep_solutions,time_sweep_data)
%FINALIZE_OUTPUTS Revalidate, analyze, plot and persist completed experiments.
folders={cfg.output.figures,cfg.output.csv,cfg.output.mat};
for i=1:numel(folders), if ~exist(folders{i},'dir'), mkdir(folders{i}); end, end
solutions={solution_min_time,solution_no_resonance,solution_10Hz};
bandReference=stage_b_info.resonance_band_reference;
for i=1:3
    solutions{i}.objectives.straightness=straightness_objective(solutions{i},path,cfg);
    [solutions{i}.objectives.vibration,solutions{i}.objectives.vibration_components]= ...
        vibration_objective(solutions{i}.modal,solutions{i}.modal_velocity, ...
        solutions{i}.acceleration,solutions{i}.jerk,solutions{i}.model,cfg);
    solutions{i}.validation=validate_solution(solutions{i},path,cfg);
    solutions{i}=attach_resonance_band_objective( ...
        solutions{i},cfg,bandReference,i==3);
    if ~solutions{i}.validation.pass
        error('POP_MOO:FinalValidation','Final solution %d failed independent validation.',i);
    end
end
solution_min_time=solutions{1}; solution_no_resonance=solutions{2}; solution_10Hz=solutions{3};
validPareto=find(pareto_data.Feasible);
if isempty(validPareto), error('POP_MOO:NoParetoSolution','No feasible Pareto solution is available.'); end
[~,minimumVibrationLocal]=min(pareto_data.Jvibration(validPareto));
minimumVibrationIndex=validPareto(minimumVibrationLocal);
solution_min_vibration=pareto_solutions{minimumVibrationIndex};
[solution_min_vibration.objectives.vibration, ...
    solution_min_vibration.objectives.vibration_components]= ...
    vibration_objective(solution_min_vibration.modal, ...
    solution_min_vibration.modal_velocity,solution_min_vibration.acceleration, ...
    solution_min_vibration.jerk, ...
    solution_min_vibration.model,cfg);
solution_min_vibration.validation=validate_solution(solution_min_vibration,path,cfg);
solution_min_vibration=attach_resonance_band_objective( ...
    solution_min_vibration,cfg,bandReference,true);
if ~solution_min_vibration.validation.pass
    error('POP_MOO:MinimumVibrationValidation', ...
        'The minimum-vibration Pareto solution failed independent validation.');
end
frequency_min=calculate_frequency_metrics(solution_min_time,cfg);
frequency_no_resonance=calculate_frequency_metrics(solution_no_resonance,cfg);
frequency_10Hz=calculate_frequency_metrics(solution_10Hz,cfg);
frequencyLabel=sprintf('%g Hz',solution_10Hz.model.mode(1).frequency);
comparison_table=build_comparison_table(solution_min_time,solution_no_resonance, ...
    solution_10Hz,frequency_min,frequency_no_resonance,frequency_10Hz,cfg);
pareto_band_data=calculate_pareto_band_metrics(pareto_solutions,pareto_data,cfg);

save_trajectory_csv(solution_min_time,fullfile(cfg.output.csv,'trajectory_min_time.csv'));
save_trajectory_csv(solution_no_resonance,fullfile(cfg.output.csv,'trajectory_no_resonance.csv'));
frequencyTag=frequency_file_tag(solution_10Hz.model.mode(1).frequency);
save_trajectory_csv(solution_10Hz,fullfile(cfg.output.csv, ...
    sprintf('trajectory_%s_resonance.csv',frequencyTag)));
save_trajectory_csv(solution_min_vibration,fullfile(cfg.output.csv,'trajectory_minimum_vibration.csv'));
writetable(comparison_table,fullfile(cfg.output.csv,'comparison_table.csv'));
writetable(pareto_data,fullfile(cfg.output.csv,'pareto_data.csv'));
writetable(pareto_band_data,fullfile(cfg.output.csv,'pareto_resonance_band_metrics.csv'));
if ~isempty(time_sweep_data)
    writetable(time_sweep_data,fullfile(cfg.output.csv, ...
        'pareto_time_sweep_summary.csv'));
end
parameter_settings=write_parameter_settings(cfg);
write_corner_tables(solutions,cfg);

representativeSolutions={solution_min_time,solution_10Hz,solution_min_vibration};
representativeLabels={'Minimum time',[frequencyLabel,' knee point'],'Minimum vibration'};
representativeFolders={'minimum_time','knee','minimum_vibration'};
representativeColors={[0.85 0.2 0.15],[0.15 0.65 0.25],[0.55 0.25 0.75]};
plot_path_comparison(path,representativeSolutions,representativeLabels,cfg);
plot_corner_zoom(path,representativeSolutions,representativeLabels,cfg);
for i=1:numel(representativeSolutions)
    plot_solution_folder(path,representativeSolutions{i},representativeLabels{i}, ...
        representativeFolders{i},pareto_data,cfg,representativeColors{i});
end
plot_tolerance_error(solution_10Hz,cfg);
plot_axis_kinematics(solution_10Hz,cfg); plot_pop_states(solution_10Hz,cfg);
plot_straightness(solution_10Hz,path,cfg);
plot_resonance_response(solution_no_resonance,solution_10Hz,cfg);
plot_frequency_spectrum(frequency_no_resonance,frequency_10Hz,cfg);
plot_constraint_usage(solution_10Hz,cfg);
plot_pareto_front(pareto_data,cfg,solution_min_time,solution_10Hz,solution_min_vibration);
plot_resonance_comparison_folder(path,solution_no_resonance,solution_10Hz,cfg);
bandPlotCfg=cfg;
bandPlotCfg.output.figures=fullfile(cfg.output.figures,'resonance_comparison');
plot_pareto_band_metrics(pareto_band_data,bandPlotCfg);
plot_time_sweep_frequency_analysis(time_sweep_solutions,time_sweep_data,cfg);

solution_resonance_suppressed=solution_10Hz;
frequency_resonance_suppressed=frequency_10Hz;
save(fullfile(cfg.output.mat,'results.mat'),'cfg','path','test_results', ...
    'solution_min_time','solution_no_resonance','solution_10Hz','solution_min_vibration', ...
    'solution_resonance_suppressed', ...
    'comparison_table','pareto_data','pareto_band_data','pareto_solutions','stage_a_search', ...
    'stage_b_info','time_sweep_solutions','time_sweep_data', ...
    'parameter_settings', ...
    'frequency_min','frequency_no_resonance','frequency_10Hz', ...
    'frequency_resonance_suppressed','-v7.3');
artifacts=struct('solution_min_time',solution_min_time, ...
    'solution_no_resonance',solution_no_resonance,'solution_10Hz',solution_10Hz, ...
    'solution_resonance_suppressed',solution_resonance_suppressed, ...
    'solution_min_vibration',solution_min_vibration, ...
    'comparison_table',comparison_table,'pareto_band_data',pareto_band_data, ...
    'parameter_settings',parameter_settings, ...
    'time_sweep_solutions',{time_sweep_solutions}, ...
    'time_sweep_data',time_sweep_data, ...
    'frequency_min',frequency_min, ...
    'frequency_no_resonance',frequency_no_resonance,'frequency_10Hz',frequency_10Hz);
end

function write_corner_tables(solutions,cfg)
frequencyLabel=sprintf('%g Hz',solutions{3}.model.mode(1).frequency);
names=["Minimum Time","No Resonance Suppression",string([frequencyLabel,' Suppression'])];
rows=table();
for k=1:numel(solutions)
    c=solutions{k}.validation.corners;
    t=struct2table(c); t.Solution=repmat(names(k),height(t),1);
    t=movevars(t,'Solution','Before',1); rows=[rows;t]; %#ok<AGROW>
end
writetable(rows,fullfile(cfg.output.csv,'corner_analysis.csv'));
end

function tag=frequency_file_tag(frequency)
tag=strrep(sprintf('%gHz',frequency),'.','p');
tag=strrep(tag,'-','m');
end
