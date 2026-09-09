function repaired = repair_control_projection(solution,path,cfg)
%REPAIR_CONTROL_PROJECTION Project POP onto exact terminal equalities/bounds.
% This deterministic polish changes the solver POP sequence by the minimum
% 2-norm amount, enforces physical |POP|<=POPMax and zero endpoint POP, then
% recomputes every shooting state by exact augmented ZOH propagation.
if exist('lsqlin','file')~=2
    repaired=solution; return
end
N=solution.N; model=build_augmented_dynamics(cfg);
Ad=model.Ad_scaled(1:6,1:6); Bd=model.Bd_scaled(1:6);
C=zeros(6,N);
for k=1:N, C(:,k)=Ad^(N-k)*Bd; end
repaired=solution; U=zeros(2,N);
opts=optimoptions('lsqlin','Display','none','Algorithm','interior-point', ...
    'ConstraintTolerance',1e-12,'OptimalityTolerance',1e-12,'MaxIterations',2000);
for axis=1:2
    x0=[path.points(1,axis)/cfg.scale.position;zeros(5,1)];
    target=x0;
    beq=target-Ad^N*x0;
    u0=solution.U(axis,:).'/cfg.limits.POPMax;
    lb=-ones(N,1); ub=ones(N,1); lb([1 end])=0; ub([1 end])=0;
    u=lsqlin(speye(N),u0,[],[],C,beq,lb,ub,u0,opts);
    U(axis,:)=u.'*cfg.limits.POPMax;
end
Xaxis=zeros(model.n_axis_state,N+1,2);
for axis=1:2
    x0=[path.points(1,axis);zeros(model.n_axis_state-1,1)];
    Xaxis(:,:,axis)=simulate_discrete_dynamics(model.Ad,model.Bd,x0,U(axis,:));
end
repaired.U=U; repaired.Xaxis=Xaxis; repaired.model=model;
repaired.position=squeeze(Xaxis(1,:,:)).';
repaired.velocity=squeeze(Xaxis(2,:,:)).';
repaired.acceleration=squeeze(Xaxis(3,:,:)).';
repaired.jerk=squeeze(Xaxis(4,:,:)).'; repaired.snap=squeeze(Xaxis(5,:,:)).';
repaired.crackle=squeeze(Xaxis(6,:,:)).';
repaired.modal=zeros(2,model.n_modes,N+1);
repaired.modal_velocity=zeros(2,model.n_modes,N+1);
for axis=1:2
    for m=1:model.n_modes
        repaired.modal(axis,m,:)=Xaxis(model.mode(m).q_index,:,axis);
        repaired.modal_velocity(axis,m,:)=Xaxis(model.mode(m).qdot_index,:,axis);
    end
end
repaired.objectives.straightness=straightness_objective(repaired,path,cfg);
[repaired.objectives.vibration,repaired.objectives.vibration_components]= ...
    vibration_objective(repaired.modal,repaired.modal_velocity, ...
    repaired.acceleration,repaired.jerk,model,cfg);
repaired.validation=validate_solution(repaired,path,cfg);
repaired.control_projection.relative_change=norm(U-solution.U,'fro')/max(norm(solution.U,'fro'),eps);
repaired.control_projection.applied=true;
repaired.feasible=false;
if repaired.validation.pass
    repaired.status=[char(solution.status),'_control_projected'];
    repaired.feasible=true;
end
end
