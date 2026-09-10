function artifacts = finalize_outputs(cfg,path,solution_min_time_reference, ...
    stage_b_seed_no_suppression,stage_b_seed_suppression,test_results, ...
    stage_a_search,stage_b_info,time_sweep_solutions,time_sweep_data)
%FINALIZE_OUTPUTS Validate, select and export the time-vibration Pareto set.
folders={cfg.output.figures,cfg.output.csv,cfg.output.mat};
for index=1:numel(folders)
    if ~exist(folders{index},'dir'), mkdir(folders{index}); end
end

[pareto_solutions,pareto_data,pareto_selection]= ...
    build_time_vibration_pareto(time_sweep_solutions,time_sweep_data);
time_sweep_data=pareto_data;
minimumTimeIndex=pareto_selection.minimum_time_index;
kneeIndex=pareto_selection.knee_index;
minimumVibrationIndex=pareto_selection.minimum_vibration_index;
minimumTimePair=time_sweep_solutions{minimumTimeIndex};
kneePair=time_sweep_solutions{kneeIndex};
minimumVibrationPair=time_sweep_solutions{minimumVibrationIndex};

candidateSolutions={minimumTimePair.suppression,kneePair.no_suppression, ...
    kneePair.suppression,minimumVibrationPair.suppression};
bandReferences={minimumTimePair.band_reference,kneePair.band_reference, ...
    kneePair.band_reference,minimumVibrationPair.band_reference};
enforceBand=[true,false,true,true];
for index=1:numel(candidateSolutions)
    solution=candidateSolutions{index};
    solution.objectives.straightness=straightness_objective(solution,path,cfg);
    [solution.objectives.vibration,solution.objectives.vibration_components]= ...
        vibration_objective(solution.modal,solution.modal_velocity, ...
        solution.acceleration,solution.jerk,solution.model,cfg);
    solution.validation=validate_solution(solution,path,cfg);
    solution=attach_resonance_band_objective( ...
        solution,cfg,bandReferences{index},enforceBand(index));
    if ~solution.validation.pass
        error('POP_MOO:FinalValidation', ...
            'Final representative solution %d failed independent validation.',index);
    end
    candidateSolutions{index}=solution;
end

solution_min_time=candidateSolutions{1};
solution_no_resonance=candidateSolutions{2};
solution_10Hz=candidateSolutions{3};
solution_min_vibration=candidateSolutions{4};
solution_min_time.name='Minimum-time Pareto solution';
solution_no_resonance.name='Knee time - No resonance suppression';
solution_10Hz.name=sprintf('Time-vibration knee - %g Hz suppression', ...
    solution_10Hz.model.mode(1).frequency);
solution_min_vibration.name='Minimum-vibration Pareto solution';

frequency_min=calculate_frequency_metrics(solution_min_time,cfg);
frequency_no_resonance=calculate_frequency_metrics(solution_no_resonance,cfg);
frequency_10Hz=calculate_frequency_metrics(solution_10Hz,cfg);
comparison_table=build_comparison_table(solution_min_time,solution_no_resonance, ...
    solution_10Hz,frequency_min,frequency_no_resonance,frequency_10Hz,cfg);
pareto_band_data=calculate_pareto_band_metrics(pareto_solutions,pareto_data,cfg);

save_trajectory_csv(solution_min_time,fullfile(cfg.output.csv,'trajectory_min_time.csv'));
save_trajectory_csv(solution_no_resonance,fullfile(cfg.output.csv,'trajectory_no_resonance.csv'));
frequencyTag=frequency_file_tag(solution_10Hz.model.mode(1).frequency);
save_trajectory_csv(solution_10Hz,fullfile(cfg.output.csv, ...
    sprintf('trajectory_%s_resonance.csv',frequencyTag)));
save_trajectory_csv(solution_min_vibration, ...
    fullfile(cfg.output.csv,'trajectory_minimum_vibration.csv'));
writetable(comparison_table,fullfile(cfg.output.csv,'comparison_table.csv'));
writetable(pareto_data,fullfile(cfg.output.csv,'pareto_data.csv'));
writetable(pareto_band_data, ...
    fullfile(cfg.output.csv,'pareto_resonance_band_metrics.csv'));
writetable(time_sweep_data, ...
    fullfile(cfg.output.csv,'pareto_time_sweep_summary.csv'));
parameter_settings=write_parameter_settings(cfg);
write_corner_tables({solution_min_time,solution_no_resonance,solution_10Hz},cfg);

representativeSolutions={solution_min_time,solution_10Hz,solution_min_vibration};
representativeLabels={'Minimum time Pareto','Time-vibration knee','Minimum vibration'};
representativeFolders={'minimum_time','knee','minimum_vibration'};
representativeColors={[0.85 0.2 0.15],[0.15 0.65 0.25],[0.55 0.25 0.75]};
plot_path_comparison(path,representativeSolutions,representativeLabels,cfg);
plot_corner_zoom(path,representativeSolutions,representativeLabels,cfg);
for index=1:numel(representativeSolutions)
    plot_solution_folder(path,representativeSolutions{index}, ...
        representativeLabels{index},representativeFolders{index}, ...
        pareto_data,cfg,representativeColors{index});
end
plot_tolerance_error(solution_10Hz,cfg);
plot_axis_kinematics(solution_10Hz,cfg);
plot_pop_states(solution_10Hz,cfg);
plot_straightness(solution_10Hz,path,cfg);
plot_resonance_response(solution_no_resonance,solution_10Hz,cfg);
plot_frequency_spectrum(frequency_no_resonance,frequency_10Hz,cfg);
plot_constraint_usage(solution_10Hz,cfg);
plot_pareto_front(pareto_data,cfg,solution_min_time, ...
    solution_10Hz,solution_min_vibration);
plot_resonance_comparison_folder(path,solution_no_resonance,solution_10Hz,cfg);
bandPlotCfg=cfg;
bandPlotCfg.output.figures=fullfile(cfg.output.figures,'resonance_comparison');
plot_pareto_band_metrics(pareto_band_data,bandPlotCfg);
plot_time_sweep_frequency_analysis(time_sweep_solutions,time_sweep_data,cfg);

solution_resonance_suppressed=solution_10Hz;
frequency_resonance_suppressed=frequency_10Hz;
save(fullfile(cfg.output.mat,'results.mat'),'cfg','path','test_results', ...
    'solution_min_time','solution_min_time_reference','solution_no_resonance', ...
    'solution_10Hz','solution_min_vibration','solution_resonance_suppressed', ...
    'stage_b_seed_no_suppression','stage_b_seed_suppression', ...
    'comparison_table','pareto_data','pareto_band_data','pareto_solutions', ...
    'pareto_selection','stage_a_search','stage_b_info', ...
    'time_sweep_solutions','time_sweep_data','parameter_settings', ...
    'frequency_min','frequency_no_resonance','frequency_10Hz', ...
    'frequency_resonance_suppressed','-v7.3');
artifacts=struct('solution_min_time',solution_min_time, ...
    'solution_min_time_reference',solution_min_time_reference, ...
    'solution_no_resonance',solution_no_resonance, ...
    'solution_10Hz',solution_10Hz, ...
    'solution_resonance_suppressed',solution_resonance_suppressed, ...
    'solution_min_vibration',solution_min_vibration, ...
    'comparison_table',comparison_table, ...
    'pareto_data',pareto_data, ...
    'pareto_solutions',{pareto_solutions}, ...
    'pareto_selection',pareto_selection, ...
    'pareto_band_data',pareto_band_data, ...
    'parameter_settings',parameter_settings, ...
    'time_sweep_solutions',{time_sweep_solutions}, ...
    'time_sweep_data',time_sweep_data, ...
    'frequency_min',frequency_min, ...
    'frequency_no_resonance',frequency_no_resonance, ...
    'frequency_10Hz',frequency_10Hz);
end

function write_corner_tables(solutions,cfg)
frequencyLabel=sprintf('%g Hz',solutions{3}.model.mode(1).frequency);
names=["Minimum-time Pareto";"Knee time - No suppression"; ...
    string([frequencyLabel,' time-vibration knee'])];
rows=table();
for index=1:numel(solutions)
    corners=solutions{index}.validation.corners;
    current=struct2table(corners);
    current.Solution=repmat(names(index),height(current),1);
    current=movevars(current,'Solution','Before',1);
    rows=[rows;current]; %#ok<AGROW>
end
writetable(rows,fullfile(cfg.output.csv,'corner_analysis.csv'));
end

function tag=frequency_file_tag(frequency)
tag=strrep(sprintf('%gHz',frequency),'.','p');
tag=strrep(tag,'-','m');
end
