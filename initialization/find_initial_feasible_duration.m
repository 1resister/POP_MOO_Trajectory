function [solution, history] = find_initial_feasible_duration(path,cfg)
%FIND_INITIAL_FEASIBLE_DURATION Grow deterministic seed until all limits pass.
base = ceil(cfg.optimization.initial_segment_time/cfg.Ts);
Nvec = max(12,ceil(base*path.lengths(:).'/max(path.lengths)));
history = table('Size',[0 8], ...
    'VariableTypes',{'double','double','double','double','double','double','double','logical'}, ...
    'VariableNames',{'Attempt','N','Vratio','Aratio','Jratio','SnapRatio','POPratio','Pass'});
for attempt=1:cfg.optimization.max_initial_attempts
    solution=generate_stop_at_corners_guess(path,cfg,Nvec);
    ratios=local_ratios(solution,cfg);
    pass=max(ratios)<=1+cfg.validation.relative_limit_tol && ...
        max(abs(solution.crackle),[],'all')<=cfg.limits.CrackleMax*(1+cfg.validation.relative_limit_tol);
    history(end+1,:)={attempt,solution.N,ratios(1),ratios(2),ratios(3),ratios(4),ratios(6),pass}; %#ok<AGROW>
    if pass
        solution.initial_limit_ratios=ratios;
        solution.feasible=true;
        return
    end
    Nvec=ceil(Nvec*cfg.optimization.initial_growth);
end
error('POP_MOO:NoInitialGuess','Could not construct a limit-feasible deterministic seed.');
end

function ratios=local_ratios(s,cfg)
ratios=[max(abs(s.velocity),[],'all')/cfg.limits.Vmax, ...
    max(abs(s.acceleration),[],'all')/cfg.limits.Amax, ...
    max(abs(s.jerk),[],'all')/cfg.limits.Jmax, ...
    max(abs(s.snap),[],'all')/cfg.limits.SnapMax, ...
    max(abs(s.crackle),[],'all')/cfg.limits.CrackleMax, ...
    max(abs(s.U),[],'all')/cfg.limits.POPMax];
end
