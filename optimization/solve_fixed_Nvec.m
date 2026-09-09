function solution = solve_fixed_Nvec(path,cfg,Nvec,warm,objectiveSpec)
%SOLVE_FIXED_NVEC Solve and independently validate one integer allocation.
if nargin<4, warm=[]; end
if nargin<5 || isempty(objectiveSpec), objectiveSpec=struct(); end
if ~isfield(objectiveSpec,'stage'), objectiveSpec.stage='feasibility'; end
if ~isfield(objectiveSpec,'lambda_vibration'), objectiveSpec.lambda_vibration=0; end
if ~isfield(objectiveSpec,'lambda_straightness'), objectiveSpec.lambda_straightness=1; end
problem=build_pop_ocp(path,cfg,Nvec,objectiveSpec);
nx=problem.model.n_axis_state;
if isempty(warm)
    warm=generate_stop_at_corners_guess(path,cfg,Nvec);
elseif ~isequal(warm.Nvec,Nvec)
    warm=resample_warm_start(warm,Nvec,cfg);
end
X0=zeros(2*nx,problem.N+1);
for axis=1:2
    rows=(axis-1)*nx+(1:nx);
    X0(rows,:)=warm.Xaxis(:,:,axis)./problem.model.scale_axis;
end
problem.opti.set_initial(problem.Xbar,X0);
problem.opti.set_initial(problem.Ubar,warm.U/cfg.scale.pop);
if ~isempty(problem.qPeak) && isfield(warm,'modal') && ~isempty(warm.modal)
    qp=max(abs(warm.modal),[],3);
    problem.opti.set_initial(problem.qPeak,max(qp,1e-9));
end
if isfield(problem,'axisPeak') && isfield(warm,'acceleration') && isfield(warm,'jerk')
    axisRms0=[sqrt(mean((warm.acceleration/cfg.limits.Amax).^2,2)), ...
        sqrt(mean((warm.jerk/cfg.limits.Jmax).^2,2))];
    axisPeak0=[max(abs(warm.acceleration),[],2)/cfg.limits.Amax, ...
        max(abs(warm.jerk),[],2)/cfg.limits.Jmax];
    problem.opti.set_initial(problem.axisRms,max(axisRms0,1e-9));
    problem.opti.set_initial(problem.axisPeak,max(axisPeak0,1e-9));
end
if ~isempty(problem.bandCoefficients) && isfield(warm,'modal') && ~isempty(warm.modal)
    coefficient0=zeros(size(problem.bandCoefficients,1), ...
        numel(problem.bandProjection.frequencies));
    for axisIndex=1:2
        q=squeeze(warm.modal(axisIndex,1,:)).';
        coefficient0(2*axisIndex-1,:)=q*problem.bandProjection.cosine.';
        coefficient0(2*axisIndex,:)=q*problem.bandProjection.sine.';
    end
    problem.opti.set_initial(problem.bandCoefficients,coefficient0);
end

solverSuccess=false; timedOut=false; message=''; stats=struct();
try
    result=problem.opti.solve();
    XbarValue=full(result.value(problem.Xbar));
    UbarValue=full(result.value(problem.Ubar));
    solverSuccess=true; stats=result.stats(); message=stats.return_status;
catch ME
    message=ME.message;
    try
        stats=problem.opti.stats();
        if isfield(stats,'return_status'), message=stats.return_status; end
        timedOut=contains(lower(message),'cpu') || contains(lower(message),'time');
        XbarValue=full(problem.opti.debug.value(problem.Xbar));
        UbarValue=full(problem.opti.debug.value(problem.Ubar));
    catch
        XbarValue=X0; UbarValue=warm.U/cfg.scale.pop;
    end
end
solution=unpack_solution(XbarValue,UbarValue,path,cfg,Nvec,problem.model);
solution.status=message; solution.solver_message=message; solution.solver_stats=stats;
solution.solver_success=solverSuccess; solution.solver_timeout=timedOut;
solution.objective_spec=objectiveSpec;
solution.objectives.straightness=straightness_objective(solution,path,cfg);
[solution.objectives.vibration,solution.objectives.vibration_components]= ...
    vibration_objective(solution.modal,solution.modal_velocity, ...
    solution.acceleration,solution.jerk,problem.model,cfg);
solution.validation=validate_solution(solution,path,cfg);
if isfield(objectiveSpec,'resonance_band_reference') && ...
        ~isempty(objectiveSpec.resonance_band_reference)
    reference.proxy=objectiveSpec.resonance_band_reference;
    if isfield(objectiveSpec,'resonance_band_reference_actual')
        reference.actual=objectiveSpec.resonance_band_reference_actual;
    else
        reference.actual=reference.proxy;
    end
    enforce=cfg.objective.resonance_band_hard_enable && ...
        objectiveSpec.lambda_vibration>0;
    solution=attach_resonance_band_objective(solution,cfg,reference,enforce);
end
solution.feasible=(solverSuccess || timedOut) && solution.validation.pass;
if timedOut && solution.validation.pass, solution.status='solver_timeout_but_feasible'; end
end

function solution=unpack_solution(Xbar,Ubar,path,cfg,Nvec,model)
N=sum(Nvec); nx=model.n_axis_state;
Xaxis=zeros(nx,N+1,2);
for axis=1:2
    rows=(axis-1)*nx+(1:nx);
    Xaxis(:,:,axis)=Xbar(rows,:).*model.scale_axis;
end
solution.Nvec=Nvec; solution.N=N; solution.T=N*cfg.Ts;
solution.time=(0:N)*cfg.Ts; solution.Xaxis=Xaxis;
solution.U=Ubar*cfg.scale.pop; solution.model=model; solution.path=path;
solution.position=squeeze(Xaxis(1,:,:)).';
solution.velocity=squeeze(Xaxis(2,:,:)).';
solution.acceleration=squeeze(Xaxis(3,:,:)).';
solution.jerk=squeeze(Xaxis(4,:,:)).';
solution.snap=squeeze(Xaxis(5,:,:)).';
solution.crackle=squeeze(Xaxis(6,:,:)).';
solution.segment_index=zeros(1,N+1); cursor=0;
for i=1:numel(Nvec)
    solution.segment_index(cursor+(1:Nvec(i)+1))=i; cursor=cursor+Nvec(i);
end
solution.modal=zeros(2,model.n_modes,N+1);
solution.modal_velocity=zeros(2,model.n_modes,N+1);
for axis=1:2
    for m=1:model.n_modes
        solution.modal(axis,m,:)=Xaxis(model.mode(m).q_index,:,axis);
        solution.modal_velocity(axis,m,:)=Xaxis(model.mode(m).qdot_index,:,axis);
    end
end
end
