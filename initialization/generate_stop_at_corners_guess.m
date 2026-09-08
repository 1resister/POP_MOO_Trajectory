function solution = generate_stop_at_corners_guess(path, cfg, Nvec)
%GENERATE_STOP_AT_CORNERS_GUESS Deterministic, exact-ZOH feasible seed.
% Each CL segment is traversed with a degree-11 smoothstep-shaped POP and
% then projected onto the exact discrete terminal constraints. This seed is
% deliberately conservative and stops at intermediate corners only as an
% initialization; the OCP does not impose those stops.
if nargin < 3 || isempty(Nvec)
    Nvec = ceil(cfg.optimization.initial_segment_time/cfg.Ts) * ...
        max(path.lengths(:).'/max(path.lengths),0.05);
    Nvec = max(ceil(Nvec),12);
end
Nvec = round(Nvec(:).');
if numel(Nvec)~=path.n_segments || any(Nvec<8)
    error('POP_MOO:BadNvec','Nvec must contain at least 8 intervals per segment.');
end
[Ad,Bd] = exact_pop_discretization(cfg.Ts);
Ntotal = sum(Nvec);
U = zeros(2,Ntotal);
cursor = 0;
for i = 1:path.n_segments
    Ni = Nvec(i); T = Ni*cfg.Ts;
    direction = path.points(i+1,1:2)-path.points(i,1:2);
    % Dimensionless exact ZOH model in normalized time tau in [0,1].
    [Aunit,Bunit] = build_pop_chain();
    Munit = expm([Aunit Bunit; zeros(1,7)]/Ni);
    Adu = Munit(1:6,1:6); Bdu = Munit(1:6,7);
    [~,smooth] = generate_high_order_smoothstep(((0:Ni-1)+0.5)/Ni);
    w0 = smooth(7,:); % dimensionless d^6q/dtau^6
    C = zeros(6,Ni);
    for k=1:Ni, C(:,k)=Adu^(Ni-k)*Bdu; end
    target = [1;zeros(5,1)];
    free = 2:Ni-1; % exact zero POP at both ends of every seed segment
    w0([1 end])=0; % endpoint-zero control is part of the projection baseline
    residual = target-C*w0(:);
    correction = lsqminnorm(C(:,free),residual,1e-13);
    w = w0; w(free)=w(free)+correction.'; w([1 end])=0;
    terminalResidual = norm(C*w(:)-target,inf);
    if terminalResidual>2e-8
        error('POP_MOO:SeedProjection','Seed endpoint projection failed (%.3g).',terminalResidual);
    end
    U(:,cursor+(1:Ni)) = direction(:)/T^6 .* w;
    cursor = cursor+Ni;
end

% Propagate the physical motion chain exactly. Modal states are propagated
% by the same augmented ZOH model and are not reset at CL junctions.
model = build_augmented_dynamics(cfg);
Xaxis = zeros(model.n_axis_state,Ntotal+1,2);
for axis=1:2
    Xaxis(:,:,axis)=simulate_discrete_dynamics(model.Ad,model.Bd, ...
        [path.points(1,axis);zeros(model.n_axis_state-1,1)],U(axis,:));
end
solution = pack_initial_solution(Xaxis,U,Nvec,path,cfg,model);
solution.status = 'initial_stop_at_corners';
solution.solver_message = 'Deterministic projected smoothstep seed';
end

function solution = pack_initial_solution(Xaxis,U,Nvec,path,cfg,model)
N = sum(Nvec);
solution.Nvec=Nvec; solution.N=N; solution.T=N*cfg.Ts;
solution.time=(0:N)*cfg.Ts;
solution.segment_index=zeros(1,N+1);
cursor=0;
for i=1:numel(Nvec)
    idx=cursor+(1:Nvec(i)+1);
    solution.segment_index(idx)=i;
    cursor=cursor+Nvec(i);
end
solution.Xaxis=Xaxis; solution.U=U; solution.model=model;
solution.position=squeeze(Xaxis(1,:,:)).';
solution.velocity=squeeze(Xaxis(2,:,:)).';
solution.acceleration=squeeze(Xaxis(3,:,:)).';
solution.jerk=squeeze(Xaxis(4,:,:)).';
solution.snap=squeeze(Xaxis(5,:,:)).';
solution.crackle=squeeze(Xaxis(6,:,:)).';
solution.path=path;
if model.n_modes>0
    solution.modal=zeros(2,model.n_modes,N+1);
    solution.modal_velocity=zeros(2,model.n_modes,N+1);
    for axis=1:2
        for m=1:model.n_modes
            solution.modal(axis,m,:)=Xaxis(model.mode(m).q_index,:,axis);
            solution.modal_velocity(axis,m,:)=Xaxis(model.mode(m).qdot_index,:,axis);
        end
    end
else
    solution.modal=zeros(2,0,N+1); solution.modal_velocity=solution.modal;
end
solution.feasible=false;
end
