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
extra=Nmax-Nmin; Nvec=solutionMin.Nvec;
% Distribute slack without imposing equality; add to currently shortest ratios.
for k=1:extra
    [~,i]=min(Nvec./path.lengths(:).'); Nvec(i)=Nvec(i)+1;
end
warm=resample_warm_start(solutionMin,Nvec,cfg);
baseStraight=max(straightness_objective(solutionMin,path,cfg),1e-8);
[baseVibration,~]=vibration_objective(solutionMin.modal,solutionMin.modal_velocity,solutionMin.model,cfg);
baseVibration=max(baseVibration,1e-8);

fprintf('Stage B: N range [%d,%d], selected N=%d (T=%.3f s).\n', ...
    Nmin,Nmax,sum(Nvec),sum(Nvec)*cfg.Ts);
specA=struct('stage','secondary','lambda_vibration',0, ...
    'lambda_straightness',1,'straightness_scale',baseStraight, ...
    'vibration_scale',baseVibration);
solutionNoRes=solve_fixed_Nvec(path,cfg,Nvec,warm,specA);
solutionNoRes.name='Near Minimum Time - No Resonance Suppression';
if ~solutionNoRes.feasible
    error('POP_MOO:SecondaryA','No-resonance secondary solution is infeasible: %s',solutionNoRes.status);
end
fprintf('  Case A feasible: straightness %.6g, vibration %.6g\n', ...
    solutionNoRes.objectives.straightness,solutionNoRes.objectives.vibration);

[paretoSolutions,paretoData]=generate_pareto_solutions(path,cfg,Nvec, ...
    solutionNoRes,baseStraight,baseVibration);
solutionRes=choose_compromise_solution(paretoSolutions,paretoData);
solutionRes.name='Near Minimum Time - 10 Hz Resonance Suppression';
info.Nmin=Nmin; info.Nmax=Nmax; info.Nvec=Nvec;
info.time_limit=(1+cfg.optimization.time_slack)*solutionMin.T;
info.discrete_exception=discreteException;
end
