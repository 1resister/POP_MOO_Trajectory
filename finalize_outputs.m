function artifacts = finalize_outputs(cfg,path,solution_min_time, ...
    solution_no_resonance,solution_10Hz,pareto_data,pareto_solutions, ...
    test_results,stage_a_search,stage_b_info)
%FINALIZE_OUTPUTS Revalidate, analyze, plot and persist completed experiments.
folders={cfg.output.figures,cfg.output.csv,cfg.output.mat};
for i=1:numel(folders), if ~exist(folders{i},'dir'), mkdir(folders{i}); end, end
solutions={solution_min_time,solution_no_resonance,solution_10Hz};
for i=1:3
    solutions{i}.validation=validate_solution(solutions{i},path,cfg);
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
solution_min_vibration.validation=validate_solution(solution_min_vibration,path,cfg);
if ~solution_min_vibration.validation.pass
    error('POP_MOO:MinimumVibrationValidation', ...
        'The minimum-vibration Pareto solution failed independent validation.');
end
frequency_min=calculate_frequency_metrics(solution_min_time,cfg);
frequency_no_resonance=calculate_frequency_metrics(solution_no_resonance,cfg);
frequency_10Hz=calculate_frequency_metrics(solution_10Hz,cfg);
comparison_table=build_comparison_table(solution_min_time,solution_no_resonance, ...
    solution_10Hz,frequency_min,frequency_no_resonance,frequency_10Hz,cfg);

save_trajectory_csv(solution_min_time,fullfile(cfg.output.csv,'trajectory_min_time.csv'));
save_trajectory_csv(solution_no_resonance,fullfile(cfg.output.csv,'trajectory_no_resonance.csv'));
save_trajectory_csv(solution_10Hz,fullfile(cfg.output.csv,'trajectory_10Hz_resonance.csv'));
save_trajectory_csv(solution_min_vibration,fullfile(cfg.output.csv,'trajectory_minimum_vibration.csv'));
writetable(comparison_table,fullfile(cfg.output.csv,'comparison_table.csv'));
writetable(pareto_data,fullfile(cfg.output.csv,'pareto_data.csv'));
write_corner_tables(solutions,cfg);

representativeSolutions={solution_min_time,solution_10Hz,solution_min_vibration};
representativeLabels={'Minimum time','Knee point','Minimum vibration'};
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

save(fullfile(cfg.output.mat,'results.mat'),'cfg','path','test_results', ...
    'solution_min_time','solution_no_resonance','solution_10Hz','solution_min_vibration', ...
    'comparison_table','pareto_data','pareto_solutions','stage_a_search', ...
    'stage_b_info','frequency_min','frequency_no_resonance','frequency_10Hz','-v7.3');
artifacts=struct('solution_min_time',solution_min_time, ...
    'solution_no_resonance',solution_no_resonance,'solution_10Hz',solution_10Hz, ...
    'solution_min_vibration',solution_min_vibration, ...
    'comparison_table',comparison_table,'frequency_min',frequency_min, ...
    'frequency_no_resonance',frequency_no_resonance,'frequency_10Hz',frequency_10Hz);
end

function write_corner_tables(solutions,cfg)
names=["Minimum Time","No Resonance Suppression","10 Hz Suppression"];
rows=table();
for k=1:numel(solutions)
    c=solutions{k}.validation.corners;
    t=struct2table(c); t.Solution=repmat(names(k),height(t),1);
    t=movevars(t,'Solution','Before',1); rows=[rows;t]; %#ok<AGROW>
end
writetable(rows,fullfile(cfg.output.csv,'corner_analysis.csv'));
end
