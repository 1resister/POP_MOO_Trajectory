function [solutionNoRes,solutionRes,paretoData,paretoSolutions,info] = ...
    solve_secondary_objectives(solutionMin,path,cfg)
%SOLVE_SECONDARY_OBJECTIVES Epsilon-constrained near-time-optimal Stage B.
% Time is selected only from integer N in [Nmin,floor(1.02*Nmin)].
Nmin=solutionMin.N;
Nmax=floor((1+cfg.optimization.time_slack)*Nmin+1e-12);
discreteException=false;
if Nmax==Nmin
    Nmax=Nmin+1; discreteException=true;
end
Nvec=stage_b_integer_allocation(solutionMin,path,Nmax);
warm=resample_warm_start(solutionMin,Nvec,cfg);
baseStraight=max(straightness_objective(solutionMin,path,cfg),1e-8);
[baseVibration,~]=vibration_objective(solutionMin.modal,solutionMin.modal_velocity, ...
    solutionMin.acceleration,solutionMin.jerk,solutionMin.model,cfg);
baseVibration=max(baseVibration,1e-8);

fprintf('Stage B: N range [%d,%d], selected N=%d (T=%.3f s).\n', ...
    Nmin,Nmax,sum(Nvec),sum(Nvec)*cfg.Ts);

% Resampling a Stage-A solution changes the integer segment durations.  At
% higher resonance frequencies, directly interpolating modal states can be
% a poor dynamically consistent initial guess.  First restore feasibility
% at the selected Stage-B grid, then optimize the secondary objectives.
bridgeSpec=struct('stage','feasibility');
bridge=solve_fixed_Nvec(path,cfg,Nvec,warm,bridgeSpec);
if ~bridge.feasible
    error('POP_MOO:SecondaryBridge', ...
        'Stage-B feasibility bridge failed: %s',bridge.status);
end
fprintf('  Stage-B feasibility bridge: %s (max violation %.3g)\n', ...
    string(bridge.status),bridge.validation.max_violation);

specA=struct('stage','secondary','lambda_vibration',0, ...
    'lambda_straightness',1,'straightness_scale',baseStraight, ...
    'vibration_scale',baseVibration);
solutionNoRes=solve_fixed_Nvec(path,cfg,Nvec,bridge,specA);
solutionNoRes.name='Near Minimum Time - No Resonance Suppression';
if ~solutionNoRes.feasible
    error('POP_MOO:SecondaryA','No-resonance secondary solution is infeasible: %s',solutionNoRes.status);
end
fprintf('  Case A feasible: straightness %.6g, vibration %.6g\n', ...
    solutionNoRes.objectives.straightness,solutionNoRes.objectives.vibration);

bandProxy=calculate_resonance_band_proxy(solutionNoRes,cfg);
bandFrequency=calculate_frequency_metrics(solutionNoRes,cfg);
bandReference=struct('proxy',bandProxy.total,'actual', ...
    bandFrequency.mode(1).band_energy.modal_x+ ...
    bandFrequency.mode(1).band_energy.modal_y);
if bandReference.proxy<=0 || bandReference.actual<=0
    error('POP_MOO:InvalidBandReference', ...
        'No-resonance reference has a non-positive resonance-band energy.');
end
solutionNoRes=attach_resonance_band_objective( ...
    solutionNoRes,cfg,bandReference,false);
baseVibration=max(solutionNoRes.objectives.vibration,1e-8);
fprintf('  Resonance-band reference: %.6g (required reduction %.2f%%)\n', ...
    bandReference.actual,100*cfg.objective.resonance_band_min_reduction);

[paretoSolutions,paretoData]=generate_pareto_solutions(path,cfg,Nvec, ...
    solutionNoRes,baseStraight,baseVibration,bandReference);
solutionRes=choose_compromise_solution(paretoSolutions,paretoData);
solutionRes.name=sprintf('Near Minimum Time - %g Hz Resonance Suppression', ...
    solutionRes.model.mode(1).frequency);
info.Nmin=Nmin; info.Nmax=Nmax; info.Nvec=Nvec;
info.time_limit=(1+cfg.optimization.time_slack)*solutionMin.T;
info.discrete_exception=discreteException;
info.bridge_status=bridge.status;
info.bridge_max_violation=bridge.validation.max_violation;
info.resonance_band_reference=bandReference;
info.resonance_band_weight=cfg.objective.resonance_band_weight;
info.resonance_band_min_reduction=cfg.objective.resonance_band_min_reduction;
end
