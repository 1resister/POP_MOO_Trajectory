function [best,history] = refine_integer_allocation(best,path,cfg,spec)
%REFINE_INTEGER_ALLOCATION Fast independent-N_i integer boundary search.
if nargin<4
    spec=struct('stage','feasibility','lambda_vibration',0,'lambda_straightness',0);
end
rows=struct('Phase',{},'Nvec',{},'N',{},'T',{},'SolverSuccess',{}, ...
    'Feasible',{},'Status',{},'MaxViolation',{});
for sweep=1:2
    startN=best.N;
    for i=1:path.n_segments
        upper=best.Nvec(i); lower=[]; step=1;
        while upper>cfg.optimization.min_segment_samples
            probe=max(cfg.optimization.min_segment_samples,upper-step);
            candidateN=best.Nvec; candidateN(i)=probe;
            candidate=solve_candidate(candidateN,best,'segment_exponential');
            if candidate.feasible
                best=candidate; upper=probe; step=2*step;
                fprintf('  Segment %d accepted N_i=%d (N=%d).\n',i,upper,best.N);
                if probe==cfg.optimization.min_segment_samples, break; end
            else
                lower=probe; break
            end
        end
        if ~isempty(lower)
            while upper-lower>1
                mid=floor((upper+lower)/2);
                candidateN=best.Nvec; candidateN(i)=mid;
                candidate=solve_candidate(candidateN,best,'segment_bisection');
                if candidate.feasible, best=candidate; upper=mid; else, lower=mid; end
            end
        end
    end
    fprintf('  Coordinate sweep %d: N=%d, Nvec=[%s]\n',sweep,best.N,num2str(best.Nvec));
    if best.N==startN, break; end
end

% Up to four nearby cycle-transfer candidates target one lower total N.
r=min(cfg.optimization.neighborhood_radius,2); d=-r:r;
[d1,d2,d3,d4]=ndgrid(d,d,d,d); delta=[d1(:),d2(:),d3(:),d4(:)];
delta=delta(sum(delta,2)==-1,:); [~,order]=sort(sum(abs(delta),2));
delta=unique(delta(order,:),'rows','stable'); delta=delta(1:min(4,size(delta,1)),:);
for c=1:size(delta,1)
    candidateN=best.Nvec+delta(c,:);
    if any(candidateN<cfg.optimization.min_segment_samples), continue; end
    candidate=solve_candidate(candidateN,best,'neighborhood');
    if candidate.feasible, best=candidate; break; end
end
history=normalize_nvec_history(struct2table(rows));

    function solution=solve_candidate(Nvec,warm,phase)
        solution=solve_fixed_Nvec(path,cfg,Nvec,warm,spec);
        rows(end+1)=struct('Phase',string(phase),'Nvec',{Nvec},'N',sum(Nvec), ...
            'T',sum(Nvec)*cfg.Ts,'SolverSuccess',solution.solver_success, ...
            'Feasible',solution.feasible,'Status',string(solution.status), ...
            'MaxViolation',solution.validation.max_violation); %#ok<AGROW>
        if solution.feasible
            latest=solution; %#ok<NASGU>
            if ~exist(cfg.output.mat,'dir'), mkdir(cfg.output.mat); end
            save(fullfile(cfg.output.mat,'stage_a_latest_feasible.mat'),'latest','-v7.3');
        end
    end
end
