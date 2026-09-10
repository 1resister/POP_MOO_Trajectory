function results = resume_pareto_time_sweep(cfg)
%RESUME_PARETO_TIME_SWEEP Continue the paired time-vibration sweep only.
root=fileparts(mfilename('fullpath'));
addpath(genpath(root));
if nargin<1, cfg=trajectory_config(); end
resultFile=fullfile(cfg.output.mat,'results.mat');
if exist(resultFile,'file')~=2
    error('POP_MOO:MissingResults', ...
        'Base results are required before resuming the time sweep: %s',resultFile);
end
data=load(resultFile);
required={'path','stage_a_search','stage_b_info'};
for index=1:numel(required)
    if ~isfield(data,required{index})
        error('POP_MOO:IncompleteResults', ...
            'Base results are missing variable %s.',required{index});
    end
end

if isfield(data,'solution_min_time_reference')
    minimumReference=data.solution_min_time_reference;
elseif isfield(data,'solution_min_time')
    minimumReference=data.solution_min_time;
else
    error('POP_MOO:IncompleteResults','Base results have no minimum-time solution.');
end
if isfield(data,'stage_b_seed_no_suppression')
    upperNo=data.stage_b_seed_no_suppression;
elseif isfield(data,'solution_no_resonance')
    upperNo=data.solution_no_resonance;
else
    error('POP_MOO:IncompleteResults','Base results have no no-suppression seed.');
end
if isfield(data,'stage_b_seed_suppression')
    upperSupp=data.stage_b_seed_suppression;
elseif isfield(data,'solution_10Hz')
    upperSupp=data.solution_10Hz;
else
    error('POP_MOO:IncompleteResults','Base results have no suppression seed.');
end

test_results=run_unit_tests(cfg);
[pairs,sweepData]=solve_all_time_vibration_sweep( ...
    minimumReference,data.path,cfg,upperNo,upperSupp,data.stage_b_info);
artifacts=finalize_outputs(cfg,data.path,minimumReference,upperNo,upperSupp, ...
    test_results,data.stage_a_search,data.stage_b_info,pairs,sweepData);

results=struct('cfg',cfg,'path',data.path,'test_results',test_results, ...
    'solution_min_time',artifacts.solution_min_time, ...
    'solution_min_time_reference',artifacts.solution_min_time_reference, ...
    'solution_no_resonance',artifacts.solution_no_resonance, ...
    'solution_10Hz',artifacts.solution_10Hz, ...
    'solution_resonance_suppressed',artifacts.solution_resonance_suppressed, ...
    'solution_min_vibration',artifacts.solution_min_vibration, ...
    'comparison_table',artifacts.comparison_table, ...
    'pareto_data',artifacts.pareto_data, ...
    'pareto_solutions',{artifacts.pareto_solutions}, ...
    'pareto_selection',artifacts.pareto_selection, ...
    'pareto_band_data',artifacts.pareto_band_data, ...
    'time_sweep_solutions',{artifacts.time_sweep_solutions}, ...
    'time_sweep_data',artifacts.time_sweep_data, ...
    'stage_a_search',data.stage_a_search,'stage_b_info',data.stage_b_info);
disp(results.pareto_data(:,{'N','Time_s','Jvibration','ParetoOptimal', ...
    'IsMinimumTime','IsKnee','IsMinimumVibration'}));
end
