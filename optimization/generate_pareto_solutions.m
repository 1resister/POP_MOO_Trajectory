function [solutions,data] = generate_pareto_solutions(path,cfg,Nvec,warm,straightScale,vibrationScale)
%GENERATE_PARETO_SOLUTIONS Weighted secondary scan under a hard time epsilon.
weights=unique(cfg.optimization.pareto_weights(:).');
solutions=cell(numel(weights),1);
rows=struct('LambdaVibration',{},'LambdaStraightness',{},'N',{},'T',{}, ...
    'Jvibration',{},'Jstraightness',{},'Feasible',{},'Status',{});
current=warm;
for i=1:numel(weights)
    lv=weights(i); ls=1-lv;
    if i==1 && lv==0
        solution=warm;
    else
        spec=struct('stage','secondary','lambda_vibration',lv, ...
            'lambda_straightness',ls,'straightness_scale',straightScale, ...
            'vibration_scale',vibrationScale);
        solution=solve_fixed_Nvec(path,cfg,Nvec,current,spec);
    end
    solutions{i}=solution;
    if solution.feasible, current=solution; end
    rows(end+1)=struct('LambdaVibration',lv,'LambdaStraightness',ls, ...
        'N',solution.N,'T',solution.T,'Jvibration',solution.objectives.vibration, ...
        'Jstraightness',solution.objectives.straightness,'Feasible',solution.feasible, ...
        'Status',string(solution.status)); %#ok<AGROW>
    fprintf('  Pareto lambda_vib=%.2f: %s, Jvib=%.6g, Jline=%.6g\n', ...
        lv,local_state(solution.feasible),solution.objectives.vibration,solution.objectives.straightness);
end
data=struct2table(rows);
end

function s=local_state(pass)
if pass, s='feasible'; else, s='infeasible'; end
end
