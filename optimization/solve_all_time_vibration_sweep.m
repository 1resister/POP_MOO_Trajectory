function [pairs,data] = solve_all_time_vibration_sweep(solutionMin,path,cfg, ...
    solutionNoRes,solutionUpperSuppression,stageBInfo)
%SOLVE_ALL_TIME_VIBRATION_SWEEP Optimize suppression at every Stage-B time.
% The sweep covers every integer N from the minimum feasible duration to
% the epsilon-constrained Stage-B upper duration.  It runs from slow to
% fast so adjacent no-suppression and suppression solutions are effective
% warm starts for the next, more difficult, shorter trajectory.
if ~cfg.optimization.time_sweep_enable
    pairs={}; data=table(); return
end

Nvalues=stageBInfo.Nmin:stageBInfo.Nmax;
pairs=cell(numel(Nvalues),1);
lambda=cfg.optimization.time_sweep_lambda_vibration;
if ~(isscalar(lambda) && isfinite(lambda) && abs(lambda-1)<=1e-12)
    error('POP_MOO:TimeParetoRequiresPureVibration', ...
        ['time_sweep_lambda_vibration must equal 1 so every fixed-time ' ...
        'candidate minimizes vibration rather than another weighted objective.']);
end
straightScale=max(straightness_objective(solutionMin,path,cfg),1e-8);
signature=sweep_signature(solutionMin,stageBInfo,cfg,lambda);
checkpointFile=fullfile(cfg.output.mat,'pareto_time_sweep_checkpoint.mat');
if cfg.optimization.time_sweep_resume && exist(checkpointFile,'file')==2
    saved=load(checkpointFile,'pairs','signature');
    if isfield(saved,'pairs') && isfield(saved,'signature') && ...
            isequaln(saved.signature,signature) && ...
            numel(saved.pairs)==numel(pairs)
        pairs=saved.pairs;
        fprintf('Recovered compatible Pareto time-sweep checkpoint.\n');
    end
end

% Reuse the already computed Stage-B upper-time solutions when possible.
upperIndex=numel(Nvalues);
upperNo=solutionNoRes;
upperSupp=solutionUpperSuppression;
previousNo=[];
previousSupp=[];
for index=upperIndex:-1:1
    targetN=Nvalues(index);
    if ~isempty(pairs{index}) && pair_is_skipped(pairs{index},targetN)
        fprintf('Time sweep N=%d (T=%.3f s): skipped [%s, checkpoint]\n', ...
            targetN,targetN*cfg.Ts,pairs{index}.failure_identifier);
        continue
    end
    if ~isempty(pairs{index}) && pair_is_valid(pairs{index},targetN,lambda)
        previousNo=pairs{index}.no_suppression;
        previousSupp=pairs{index}.suppression;
        fprintf(['Time sweep N=%d (T=%.3f s): band reduction %.2f%% ' ...
            '[checkpoint]\n'],targetN,targetN*cfg.Ts, ...
            band_reduction(previousSupp,pairs{index}.band_reference));
        continue
    end
    if index==upperIndex
        noWarm=upperNo;
        suppressionWarm=upperSupp;
        reuseUpper=true;
    else
        noWarm=previousNo;
        suppressionWarm=previousSupp;
        reuseUpper=false;
    end
    try
        [noSuppression,suppression,reference]=solve_pair(targetN, ...
            noWarm,suppressionWarm,solutionMin,path,cfg,straightScale,lambda,reuseUpper);
    catch ME
        if is_recoverable_pair_failure(ME)
            pairs{index}=make_skipped_pair(targetN,cfg,ME);
            save(checkpointFile,'pairs','signature','-v7.3');
            fprintf(2,['Time sweep N=%d (T=%.3f s) was skipped after all ' ...
                'recovery attempts: %s\n'],targetN,targetN*cfg.Ts,ME.message);
            continue
        end
        rethrow(ME)
    end
    pairs{index}=make_pair(noSuppression,suppression,reference,lambda);
    previousNo=noSuppression;
    previousSupp=suppression;
    save(checkpointFile,'pairs','signature','-v7.3');
    suffix='';
    if reuseUpper, suffix=' [reused Stage B]'; end
    fprintf('Time sweep N=%d (T=%.3f s): band reduction %.2f%%%s\n', ...
        targetN,targetN*cfg.Ts,band_reduction(suppression,reference),suffix);
end

completed=false(size(pairs));
skipped=false(size(pairs));
for index=1:numel(pairs)
    completed(index)=pair_is_valid(pairs{index},Nvalues(index),lambda);
    skipped(index)=pair_is_skipped(pairs{index},Nvalues(index));
end
write_skipped_points(pairs(skipped),cfg);
pairs=pairs(completed);
if isempty(pairs)
    error('POP_MOO:NoCompletedTimeSweepPairs', ...
        'No feasible time-vibration pairs remained after the time sweep.');
end
rows=cellfun(@pair_row,pairs,'UniformOutput',false);
data=struct2table([rows{:}]);
end

function [noSuppression,suppression,reference]=solve_pair(targetN,noWarm, ...
    suppressionWarm,solutionMin,path,cfg,straightScale,lambda,reuseUpper)
Nvec=stage_b_integer_allocation(solutionMin,path,targetN);
solveCfg=cfg;
if targetN==solutionMin.N
    solveCfg.solver.ipopt.max_cpu_time=cfg.solver.ipopt.max_cpu_time* ...
        cfg.optimization.time_sweep_boundary_cpu_multiplier;
end
if reuseUpper && isequal(noWarm.Nvec,Nvec) && noWarm.feasible
    noSuppression=noWarm;
else
    if targetN==solutionMin.N && isequal(solutionMin.Nvec,Nvec) && ...
            solutionMin.validation.pass
        bridge=solutionMin;
    else
        bridge=solve_with_cpu_retry(path,solveCfg,Nvec,noWarm, ...
            struct('stage','feasibility'),targetN,'feasibility bridge');
    end
    if ~bridge.feasible
        error('POP_MOO:TimeSweepBridge', ...
            'Time-sweep feasibility bridge failed at N=%d: %s',targetN,bridge.status);
    end
    specNo=struct('stage','secondary', ...
        'lambda_vibration',0,'lambda_straightness',1, ...
        'straightness_scale',straightScale,'vibration_scale',1);
    noSuppression=solve_with_cpu_retry(path,solveCfg,Nvec,bridge, ...
        specNo,targetN,'no-suppression optimization');
    if ~noSuppression.feasible
        if targetN==solutionMin.N && solutionMin.validation.pass
            noSuppression=solutionMin;
            noSuppression.status='minimum_time_feasible_reference';
            noSuppression.feasible=true;
        else
            error('POP_MOO:TimeSweepNoSuppression', ...
                'No-suppression solve failed at N=%d: %s',targetN,noSuppression.status);
        end
    end
end
noSuppression.name=sprintf('T=%.3f s - No Resonance Suppression',targetN*cfg.Ts);

proxy=calculate_resonance_band_proxy(noSuppression,cfg);
frequency=calculate_frequency_metrics(noSuppression,cfg);
reference=struct('proxy',proxy.total,'actual',modal_band_energy(frequency));
if reference.proxy<=0 || reference.actual<=0
    error('POP_MOO:TimeSweepBandReference', ...
        'Non-positive resonance-band reference at N=%d.',targetN);
end
noSuppression=attach_resonance_band_objective(noSuppression,cfg,reference,false);

canReuse=reuseUpper && ~isempty(suppressionWarm) && ...
    isequal(suppressionWarm.Nvec,Nvec) && suppressionWarm.feasible;
if canReuse
    suppression=suppressionWarm;
else
    if isempty(suppressionWarm), suppressionWarm=noSuppression; end
    vibrationScale=max(noSuppression.objectives.vibration,1e-8);
    specSuppression=struct('stage','secondary', ...
        'lambda_vibration',lambda,'lambda_straightness',1-lambda, ...
        'straightness_scale',straightScale,'vibration_scale',vibrationScale, ...
        'resonance_band_reference',reference.proxy, ...
        'resonance_band_reference_actual',reference.actual);
    suppression=solve_with_cpu_retry(path,solveCfg,Nvec,suppressionWarm, ...
        specSuppression,targetN,'suppression optimization');
    if ~suppression.feasible
        % A reference trajectory can occasionally be a better interior
        % initial guess than the resampled adjacent suppressed solution.
        suppression=solve_with_cpu_retry(path,solveCfg,Nvec,noSuppression, ...
            specSuppression,targetN,'suppression fallback');
    end
    if ~suppression.feasible
        suppression=solve_suppression_homotopy(path,solveCfg,Nvec, ...
            suppressionWarm,noSuppression,specSuppression,targetN);
    end
    if ~suppression.feasible
        error('POP_MOO:TimeSweepSuppression', ...
            'Suppression solve failed at N=%d: %s',targetN,suppression.status);
    end
end
suppression.name=sprintf('T=%.3f s - %g Hz Resonance Suppression', ...
    targetN*cfg.Ts,suppression.model.mode(1).frequency);
% Reattach with the current-time reference even when the upper solution was
% reused from the original Pareto scan.
suppression=attach_resonance_band_objective(suppression,cfg,reference,true);
if ~suppression.validation.pass
    error('POP_MOO:TimeSweepValidation', ...
        'Independent band validation failed at N=%d.',targetN);
end
end

function solution=solve_with_cpu_retry(path,cfg,Nvec,warm,objectiveSpec, ...
    targetN,label)
%SOLVE_WITH_CPU_RETRY Continue once from IPOPT's last iterate after timeout.
solution=solve_fixed_Nvec(path,cfg,Nvec,warm,objectiveSpec);
if solution.feasible || ~isfield(solution,'solver_timeout') || ...
        ~solution.solver_timeout
    return
end
retryCfg=cfg;
multiplier=max(1,cfg.optimization.time_sweep_boundary_cpu_multiplier);
retryCfg.solver.ipopt.max_cpu_time=max( ...
    cfg.solver.ipopt.max_cpu_time+1, ...
    cfg.solver.ipopt.max_cpu_time*multiplier);
fprintf(['  Time sweep N=%d: %s reached the %.0f s CPU limit; ' ...
    'retrying from the last iterate with %.0f s.\n'],targetN,label, ...
    cfg.solver.ipopt.max_cpu_time,retryCfg.solver.ipopt.max_cpu_time);
solution=solve_fixed_Nvec(path,retryCfg,Nvec,solution,objectiveSpec);
end

function solution=solve_suppression_homotopy(path,cfg,Nvec,warm, ...
    noSuppression,objectiveSpec,targetN)
%SOLVE_SUPPRESSION_HOMOTOPY Introduce the hard band reduction gradually.
softCfg=cfg;
softCfg.objective.resonance_band_hard_enable=false;
fprintf('  Time sweep N=%d: starting resonance-band homotopy recovery.\n',targetN);
solution=solve_with_cpu_retry(path,softCfg,Nvec,warm,objectiveSpec, ...
    targetN,'soft-band homotopy');
if ~solution.feasible
    solution=solve_with_cpu_retry(path,softCfg,Nvec,noSuppression, ...
        objectiveSpec,targetN,'soft-band homotopy fallback');
end
if ~solution.feasible
    return
end

requiredReduction=cfg.objective.resonance_band_min_reduction;
if requiredReduction<=0
    return
end
for fraction=[0.25 0.50 0.75 1.00]
    stageCfg=cfg;
    stageCfg.objective.resonance_band_min_reduction= ...
        fraction*requiredReduction;
    fprintf('  Time sweep N=%d: homotopy requires %.2f%% band reduction.\n', ...
        targetN,100*stageCfg.objective.resonance_band_min_reduction);
    solution=solve_with_cpu_retry(path,stageCfg,Nvec,solution,objectiveSpec, ...
        targetN,'hard-band homotopy');
    if ~solution.feasible
        return
    end
end
end

function valid=pair_is_valid(pair,targetN,lambda)
valid=~isempty(pair) && isstruct(pair) && isfield(pair,'N') && pair.N==targetN && ...
    isfield(pair,'lambda_vibration') && ...
    abs(pair.lambda_vibration-lambda)<=1e-12 && ...
    isfield(pair,'no_suppression') && pair.no_suppression.feasible && ...
    isfield(pair,'suppression') && pair.suppression.feasible && ...
    pair.suppression.validation.pass;
end

function skipped=pair_is_skipped(pair,targetN)
skipped=~isempty(pair) && isstruct(pair) && isfield(pair,'N') && ...
    pair.N==targetN && isfield(pair,'skipped') && pair.skipped;
end

function recoverable=is_recoverable_pair_failure(ME)
recoverable=any(strcmp(ME.identifier,{ ...
    'POP_MOO:TimeSweepBridge', ...
    'POP_MOO:TimeSweepNoSuppression', ...
    'POP_MOO:TimeSweepSuppression', ...
    'POP_MOO:TimeSweepValidation'}));
end

function pair=make_skipped_pair(targetN,cfg,ME)
pair=struct('N',targetN,'T',targetN*cfg.Ts,'skipped',true, ...
    'failure_identifier',string(ME.identifier), ...
    'failure_message',string(ME.message));
end

function write_skipped_points(skippedPairs,cfg)
count=numel(skippedPairs);
N=zeros(count,1); Time_s=zeros(count,1);
FailureIdentifier=strings(count,1); FailureMessage=strings(count,1);
for index=1:count
    N(index)=skippedPairs{index}.N;
    Time_s(index)=skippedPairs{index}.T;
    FailureIdentifier(index)=skippedPairs{index}.failure_identifier;
    FailureMessage(index)=skippedPairs{index}.failure_message;
end
failures=table(N,Time_s,FailureIdentifier,FailureMessage);
writetable(failures,fullfile(cfg.output.csv,'pareto_time_sweep_skipped.csv'));
if count>0
    fprintf(2,['Time sweep completed with %d skipped point(s). See ' ...
        'pareto_time_sweep_skipped.csv.\n'],count);
end
end

function signature=sweep_signature(solutionMin,stageBInfo,cfg,lambda)
mode=cfg.resonance.mode(1);
signature=struct('Nmin',stageBInfo.Nmin,'Nmax',stageBInfo.Nmax, ...
    'minimum_Nvec',solutionMin.Nvec,'Ts',cfg.Ts,'lambda',lambda, ...
    'frequency',mode.frequency,'zeta',mode.zeta,'gain',mode.gain, ...
    'bandwidth',cfg.frequency.bandwidth, ...
    'band_weight',cfg.objective.resonance_band_weight, ...
    'hard_enable',cfg.objective.resonance_band_hard_enable, ...
    'minimum_reduction',cfg.objective.resonance_band_min_reduction, ...
    'straightness_hard_enable',cfg.straightness.hard_enable);
end

function pair=make_pair(noSuppression,suppression,reference,lambda)
pair=struct('N',noSuppression.N,'T',noSuppression.T, ...
    'Nvec',noSuppression.Nvec,'lambda_vibration',lambda, ...
    'no_suppression',noSuppression,'suppression',suppression, ...
    'band_reference',reference);
end

function row=pair_row(pair)
no=pair.no_suppression;
supp=pair.suppression;
noComponents=no.objectives.vibration_components;
suppComponents=supp.objectives.vibration_components;
noBand=pair.band_reference.actual;
suppBand=supp.validation.resonance_band.actual_total;
row=struct('N',pair.N,'Time_s',pair.T,'Nvec',string(mat2str(pair.Nvec)), ...
    'LambdaVibration',pair.lambda_vibration, ...
    'NoSuppressionJvibration',no.objectives.vibration, ...
    'SuppressedJvibration',supp.objectives.vibration, ...
    'NoSuppressionJstraightness',no.objectives.straightness, ...
    'SuppressedJstraightness',supp.objectives.straightness, ...
    'NoSuppressionJvibrationTimeDomain',no.objectives.vibration_time_domain, ...
    'SuppressedJvibrationTimeDomain',supp.objectives.vibration_time_domain, ...
    'NoSuppressionModalEnergy',noComponents.energy, ...
    'SuppressedModalEnergy',suppComponents.energy, ...
    'NoSuppressionModalPeakSquared',noComponents.peak_squared, ...
    'SuppressedModalPeakSquared',suppComponents.peak_squared, ...
    'NoSuppressionAccelerationRMSX',noComponents.acceleration_rms_axis(1), ...
    'NoSuppressionAccelerationRMSY',noComponents.acceleration_rms_axis(2), ...
    'SuppressedAccelerationRMSX',suppComponents.acceleration_rms_axis(1), ...
    'SuppressedAccelerationRMSY',suppComponents.acceleration_rms_axis(2), ...
    'NoSuppressionAccelerationPeakX',noComponents.acceleration_peak_axis(1), ...
    'NoSuppressionAccelerationPeakY',noComponents.acceleration_peak_axis(2), ...
    'SuppressedAccelerationPeakX',suppComponents.acceleration_peak_axis(1), ...
    'SuppressedAccelerationPeakY',suppComponents.acceleration_peak_axis(2), ...
    'NoSuppressionJerkRMSX',noComponents.jerk_rms_axis(1), ...
    'NoSuppressionJerkRMSY',noComponents.jerk_rms_axis(2), ...
    'SuppressedJerkRMSX',suppComponents.jerk_rms_axis(1), ...
    'SuppressedJerkRMSY',suppComponents.jerk_rms_axis(2), ...
    'NoSuppressionJerkPeakX',noComponents.jerk_peak_axis(1), ...
    'NoSuppressionJerkPeakY',noComponents.jerk_peak_axis(2), ...
    'SuppressedJerkPeakX',suppComponents.jerk_peak_axis(1), ...
    'SuppressedJerkPeakY',suppComponents.jerk_peak_axis(2), ...
    'NoSuppressionResonanceBandContribution', ...
        noComponents.resonance_band_contribution, ...
    'SuppressedResonanceBandContribution', ...
        suppComponents.resonance_band_contribution, ...
    'NoSuppressionBandEnergy',noBand, ...
    'SuppressedBandEnergy',suppBand, ...
    'BandReductionPercent',100*(noBand-suppBand)/noBand, ...
    'NoSuppressionMaxContourError_mm',max(no.validation.contour.error), ...
    'SuppressedMaxContourError_mm',max(supp.validation.contour.error), ...
    'NoSuppressionFeasible',no.feasible, ...
    'SuppressionFeasible',supp.feasible, ...
    'NoSuppressionStatus',string(no.status), ...
    'SuppressionStatus',string(supp.status));
end

function reduction=band_reduction(solution,reference)
reduction=100*(reference.actual- ...
    solution.validation.resonance_band.actual_total)/reference.actual;
end

function energy=modal_band_energy(frequency)
energy=frequency.mode(1).band_energy.modal_x+ ...
    frequency.mode(1).band_energy.modal_y;
end
