function [best,search] = find_minimum_integer_time(path,cfg)
%FIND_MINIMUM_INTEGER_TIME Outer integer search + warm-started feasibility NLP.
% Strategy: deterministic feasible seed -> time-compression continuation ->
% elementwise integer bisection -> single-segment coordinate descent -> a
% small allocation neighborhood. N is never a continuous IPOPT variable.
fprintf('Stage A: constructing a deterministic feasible slow trajectory...\n');
[seed,seedHistory]=find_initial_feasible_duration(path,cfg);

% Reduce the stop-at-corners seed cheaply before invoking IPOPT.
seedBest=seed;
while true
    candidateN=max(cfg.optimization.min_segment_samples, ...
        floor(seedBest.Nvec*cfg.optimization.continuation_factor));
    if isequal(candidateN,seedBest.Nvec), break; end
    candidate=generate_stop_at_corners_guess(path,cfg,candidateN);
    candidate.validation=validate_solution(candidate,path,cfg);
    if candidate.validation.pass
        seedBest=candidate;
    else
        break
    end
end
spec=struct('stage','feasibility','lambda_vibration',0,'lambda_straightness',0);
history=struct('Attempt',{},'Phase',{},'Nvec',{},'N',{},'T',{}, ...
    'SolverSuccess',{},'Feasible',{},'Status',{},'MaxViolation',{});
attempt=0;
best=solve_one(seedBest.Nvec,seedBest,'initial_nlp');
if ~best.feasible
    % Grow conservatively if numerical optimization rejected an exact seed.
    for grow=1:cfg.optimization.max_initial_attempts
        seedBest=generate_stop_at_corners_guess(path,cfg,ceil(seedBest.Nvec*cfg.optimization.initial_growth));
        best=solve_one(seedBest.Nvec,seedBest,'initial_growth');
        if best.feasible, break; end
    end
end
if ~best.feasible
    error('POP_MOO:NoFeasibleNLP','Unable to establish a solver-validated feasible fixed-N trajectory.');
end
fprintf('  Initial NLP feasible: N=%d, T=%.3f s, Nvec=[%s]\n', ...
    best.N,best.T,num2str(best.Nvec));

% Multiplicative continuation until the first infeasible lower allocation.
lowerInfeasible=[];
while true
    candidateN=max(cfg.optimization.min_segment_samples, ...
        floor(best.Nvec*cfg.optimization.continuation_factor));
    if isequal(candidateN,best.Nvec), break; end
    candidate=solve_one(candidateN,best,'continuation');
    fprintf('  Continuation N=%d: %s\n',candidate.N,ternary(candidate.feasible,'feasible','infeasible'));
    if candidate.feasible
        best=candidate;
    else
        lowerInfeasible=candidateN;
        break
    end
end

% Integer bisection between the last feasible and first infeasible vectors.
if ~isempty(lowerInfeasible)
    upperFeasible=best.Nvec;
    while sum(upperFeasible-lowerInfeasible)>1
        candidateN=floor((upperFeasible+lowerInfeasible)/2);
        if isequal(candidateN,lowerInfeasible) || isequal(candidateN,upperFeasible), break; end
        candidate=solve_one(candidateN,best,'integer_bisection');
        fprintf('  Bisection N=%d: %s\n',candidate.N,ternary(candidate.feasible,'feasible','infeasible'));
        if candidate.feasible
            best=candidate; upperFeasible=candidateN;
        else
            lowerInfeasible=candidateN;
        end
    end
end

% Independent per-segment refinement uses an exponential decrement starting
% at one cycle, followed by exact integer bisection and a small neighborhood.
[best,refineHistory]=refine_integer_allocation(best,path,cfg,spec);
refineHistory=normalize_nvec_history(refineHistory);
for k=1:height(refineHistory)
    attempt=attempt+1;
    if iscell(refineHistory.Nvec)
        candidateNvec=refineHistory.Nvec{k};
    else
        candidateNvec=refineHistory.Nvec(k,:);
    end
    history(end+1)=struct('Attempt',attempt,'Phase',refineHistory.Phase(k), ...
        'Nvec',{candidateNvec},'N',refineHistory.N(k), ...
        'T',refineHistory.T(k),'SolverSuccess',refineHistory.SolverSuccess(k), ...
        'Feasible',refineHistory.Feasible(k),'Status',refineHistory.Status(k), ...
        'MaxViolation',refineHistory.MaxViolation(k)); %#ok<AGROW>
end

search.history=normalize_nvec_history(struct2table(history));
search.seed_history=seedHistory;
search.minimum_N=best.N; search.minimum_T=best.T; search.minimum_Nvec=best.Nvec;
search.warm_start='primal X/U; Opti dual warm start is version-dependent and not exported';
if ~exist(cfg.output.mat,'dir'), mkdir(cfg.output.mat); end
save(fullfile(cfg.output.mat,'stage_a_checkpoint.mat'),'best','search','-v7.3');
fprintf('Minimum feasible integer allocation: N=%d, T=%.3f s, Nvec=[%s]\n', ...
    best.N,best.T,num2str(best.Nvec));

    function solution=solve_one(Nvec,warm,phase)
        attempt=attempt+1;
        solution=solve_fixed_Nvec(path,cfg,Nvec,warm,spec);
        history(end+1)=struct('Attempt',attempt,'Phase',string(phase), ...
            'Nvec',{Nvec},'N',sum(Nvec),'T',sum(Nvec)*cfg.Ts, ...
            'SolverSuccess',solution.solver_success,'Feasible',solution.feasible, ...
            'Status',string(solution.status),'MaxViolation',solution.validation.max_violation); %#ok<AGROW>
        if solution.feasible
            latest=solution; %#ok<NASGU>
            if ~exist(cfg.output.mat,'dir'), mkdir(cfg.output.mat); end
            save(fullfile(cfg.output.mat,'stage_a_latest_feasible.mat'),'latest','-v7.3');
        end
    end
end

function value=ternary(condition,a,b)
if condition, value=a; else, value=b; end
end
