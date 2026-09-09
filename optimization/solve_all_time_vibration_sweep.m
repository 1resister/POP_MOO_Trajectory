function [pairs,data] = solve_all_time_vibration_sweep(solutionMin,path,cfg, ...
    solutionNoRes,paretoSolutions,paretoData,stageBInfo)
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
if ~(isscalar(lambda) && isfinite(lambda) && lambda>0 && lambda<=1)
    error('POP_MOO:BadTimeSweepWeight', ...
        'time_sweep_lambda_vibration must be in the interval (0,1].');
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
paretoIndex=find(abs(paretoData.LambdaVibration-lambda)<=1e-12 & ...
    paretoData.Feasible,1,'first');
if isempty(paretoIndex)
    upperSupp=[];
else
    upperSupp=paretoSolutions{paretoIndex};
end
previousNo=[];
previousSupp=[];
for index=upperIndex:-1:1
    targetN=Nvalues(index);
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
    [noSuppression,suppression,reference]=solve_pair(targetN, ...
        noWarm,suppressionWarm,solutionMin,path,cfg,straightScale,lambda,reuseUpper);
    pairs{index}=make_pair(noSuppression,suppression,reference,lambda);
    previousNo=noSuppression;
    previousSupp=suppression;
    save(checkpointFile,'pairs','signature','-v7.3');
    suffix='';
    if reuseUpper, suffix=' [reused Stage B]'; end
    fprintf('Time sweep N=%d (T=%.3f s): band reduction %.2f%%%s\n', ...
        targetN,targetN*cfg.Ts,band_reduction(suppression,reference),suffix);
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
        bridge=solve_fixed_Nvec(path,solveCfg,Nvec,noWarm,struct('stage','feasibility'));
    end
    if ~bridge.feasible
        error('POP_MOO:TimeSweepBridge', ...
            'Time-sweep feasibility bridge failed at N=%d: %s',targetN,bridge.status);
    end
    specNo=struct('stage','secondary', ...
        'lambda_vibration',0,'lambda_straightness',1, ...
        'straightness_scale',straightScale,'vibration_scale',1);
    noSuppression=solve_fixed_Nvec(path,solveCfg,Nvec,bridge,specNo);
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
    suppression=solve_fixed_Nvec(path,solveCfg,Nvec,suppressionWarm,specSuppression);
    if ~suppression.feasible
        % A reference trajectory can occasionally be a better interior
        % initial guess than the resampled adjacent suppressed solution.
        suppression=solve_fixed_Nvec(path,solveCfg,Nvec,noSuppression,specSuppression);
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

function valid=pair_is_valid(pair,targetN,lambda)
valid=isstruct(pair) && isfield(pair,'N') && pair.N==targetN && ...
    isfield(pair,'lambda_vibration') && ...
    abs(pair.lambda_vibration-lambda)<=1e-12 && ...
    isfield(pair,'no_suppression') && pair.no_suppression.feasible && ...
    isfield(pair,'suppression') && pair.suppression.feasible && ...
    pair.suppression.validation.pass;
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
noBand=pair.band_reference.actual;
suppBand=supp.validation.resonance_band.actual_total;
row=struct('N',pair.N,'Time_s',pair.T,'Nvec',string(mat2str(pair.Nvec)), ...
    'LambdaVibration',pair.lambda_vibration, ...
    'NoSuppressionJvibration',no.objectives.vibration, ...
    'SuppressedJvibration',supp.objectives.vibration, ...
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
