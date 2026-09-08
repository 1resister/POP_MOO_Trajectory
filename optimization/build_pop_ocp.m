function problem = build_pop_ocp(path,cfg,Nvec,objectiveSpec)
%BUILD_POP_OCP Scaled CasADi direct-multiple-shooting POP OCP.
% Every 1 ms interval owns one piecewise-constant POP input. Position,
% velocity, acceleration, jerk, snap, crackle and modal states are shooting
% variables linked by the exact augmented ZOH map.
if nargin<4 || isempty(objectiveSpec), objectiveSpec=struct(); end
if ~isfield(objectiveSpec,'stage'), objectiveSpec.stage='feasibility'; end
if ~isfield(objectiveSpec,'lambda_vibration'), objectiveSpec.lambda_vibration=0; end
if ~isfield(objectiveSpec,'lambda_straightness'), objectiveSpec.lambda_straightness=1; end
if ~isfield(objectiveSpec,'vibration_scale'), objectiveSpec.vibration_scale=1; end
if ~isfield(objectiveSpec,'straightness_scale'), objectiveSpec.straightness_scale=1; end
import casadi.*
Nvec=round(Nvec(:).'); N=sum(Nvec);
model=build_augmented_dynamics(cfg); nx=model.n_axis_state;
opti=Opti();
Xbar=opti.variable(2*nx,N+1);
Ubar=opti.variable(2,N);

% Exact scaled dynamics for both axes, vectorized over all intervals.
opti.subject_to(Xbar(1:nx,2:end)==model.Ad_scaled*Xbar(1:nx,1:end-1)+model.Bd_scaled*Ubar(1,:));
opti.subject_to(Xbar(nx+(1:nx),2:end)==model.Ad_scaled*Xbar(nx+(1:nx),1:end-1)+model.Bd_scaled*Ubar(2,:));

% Per-axis hard kinematic constraints in normalized coordinates.
if isfield(cfg.solver,'hard_bound_margin')
    normalizedLimit=1-cfg.solver.hard_bound_margin;
else
    normalizedLimit=1;
end
for axis=0:1
    base=axis*nx;
    for state=2:6
        opti.subject_to(Xbar(base+state,:).'>=-normalizedLimit);
        opti.subject_to(Xbar(base+state,:).'<=normalizedLimit);
    end
end
opti.subject_to(Ubar(:)>=-normalizedLimit); opti.subject_to(Ubar(:)<=normalizedLimit);

% True start/end motion boundary conditions. Modal states start at rest but
% are not artificially forced to settle at the cycle end.
p0=path.points(1,1:2)/cfg.scale.position;
opti.subject_to(Xbar(1,1)==p0(1)); opti.subject_to(Xbar(nx+1,1)==p0(2));
opti.subject_to(Xbar(1,end)==p0(1)); opti.subject_to(Xbar(nx+1,end)==p0(2));
opti.subject_to(Xbar(2:6,1)==0); opti.subject_to(Xbar(nx+(2:6),1)==0);
opti.subject_to(Xbar(2:6,end)==0); opti.subject_to(Xbar(nx+(2:6),end)==0);
if nx>6
    opti.subject_to(Xbar(7:nx,1)==0); opti.subject_to(Xbar(nx+(7:nx),1)==0);
end
if cfg.boundary.zero_endpoint_pop
    opti.subject_to(Ubar(:,1)==0); opti.subject_to(Ubar(:,end)==0);
end

px=cfg.scale.position*Xbar(1,:); py=cfg.scale.position*Xbar(nx+1,:);
vx=cfg.scale.velocity*Xbar(2,:); vy=cfg.scale.velocity*Xbar(nx+2,:);
straight=0; cursor=0;
for i=1:path.n_segments
    Ni=Nvec(i); idx=cursor+(1:Ni+1);
    seg=path.segment(i); t=seg.tangent(1:2); n=seg.normal(1:2);
    dx=px(idx)-seg.p0(1); dy=py(idx)-seg.p0(2);
    tau=t(1)*dx+t(2)*dy;
    normalError=n(1)*dx+n(2)*dy;
    opti.subject_to(tau(:)>=-cfg.geometry.overlap_extension);
    opti.subject_to(tau(:)<=seg.length+cfg.geometry.overlap_extension);
    dtau=tau(2:end)-tau(1:end-1);
    opti.subject_to(dtau(:)>=-cfg.validation.geometry_abs_tol);
    tangentVelocity=t(1)*vx(idx)+t(2)*vy(idx);
    opti.subject_to(tangentVelocity(:)>=-cfg.validation.backward_velocity_tol);
    dCorner=fmin(fmax(tau,0),fmax(seg.length-tau,0));
    xi=fmin(1,fmax(0,1-dCorner/cfg.geometry.transition_length));
    h=3*xi.^2-2*xi.^3;
    epsAllowed=cfg.geometry.line_tolerance + ...
        (cfg.geometry.corner_tolerance-cfg.geometry.line_tolerance)*h;
    opti.subject_to(normalError(:).^2<=epsAllowed(:).^2);

    nominal=linspace(0,seg.length,Ni+1);
    dn=min(nominal,seg.length-nominal);
    alpha=min(1,max(0,dn/cfg.geometry.transition_length));
    wline=3*alpha.^2-2*alpha.^3;
    vperp=(n(1)*vx(idx)+n(2)*vy(idx))/cfg.limits.Vmax;
    straight=straight+cfg.Ts*sum(wline.*(vperp.^2));
    if cfg.straightness.hard_enable
        core=find(dn>=cfg.geometry.transition_length-1e-12);
        opti.subject_to(vperp(core)==0);
    end
    cursor=cursor+Ni;
end

% Modal energy and epigraph peak. q is dimensionless; qdot is normalized by wn.
vibration=0; qPeak=[];
if model.n_modes>0
    qPeak=opti.variable(2,model.n_modes);
    opti.subject_to(qPeak(:)>=0);
    for axis=0:1
        base=axis*nx;
        for m=1:model.n_modes
            q=Xbar(base+model.mode(m).q_index,:); % q scale is 1
            qdBar=Xbar(base+model.mode(m).qdot_index,:); % qdot/wn
            energy=sum(q.^2+qdBar.^2)/(N+1);
            opti.subject_to(qPeak(axis+1,m)>=q(:));
            opti.subject_to(qPeak(axis+1,m)>=-q(:));
            vibration=vibration+cfg.objective.vibration_energy_weight*energy + ...
                cfg.objective.vibration_peak_weight*qPeak(axis+1,m)^2;
        end
    end
end
regularization=regularization_objective(Xbar,Ubar,cfg,nx);
switch lower(objectiveSpec.stage)
    case 'feasibility'
        objective=regularization;
    case 'secondary'
        objective=objectiveSpec.lambda_straightness*straight/max(objectiveSpec.straightness_scale,1e-10) + ...
            objectiveSpec.lambda_vibration*vibration/max(objectiveSpec.vibration_scale,1e-10) + regularization;
    otherwise
        error('POP_MOO:BadObjectiveStage','Unknown objective stage %s.',objectiveSpec.stage);
end
opti.minimize(objective);
opti.solver('ipopt',configure_ipopt(cfg));

problem.opti=opti; problem.Xbar=Xbar; problem.Ubar=Ubar;
problem.qPeak=qPeak; problem.model=model; problem.N=N; problem.Nvec=Nvec;
problem.objective=objective; problem.straightness=straight;
problem.vibration=vibration; problem.regularization=regularization;
problem.objectiveSpec=objectiveSpec;
end
