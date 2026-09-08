function model = build_augmented_dynamics(cfg)
%BUILD_AUGMENTED_DYNAMICS Exact joint POP-chain/modal ZOH model per axis.
modes = cfg.resonance.mode([cfg.resonance.mode.enable]);
nMode = numel(modes);
n = 6+2*nMode;
[Apop,Bpop] = build_pop_chain();
A = zeros(n);
B = zeros(n,1);
A(1:6,1:6) = Apop;
B(1:6) = Bpop;
modeInfo = repmat(struct('frequency',[],'wn',[],'zeta',[],'gain',[], ...
    'q_index',[],'qdot_index',[]),nMode,1);
for m = 1:nMode
    rows = 6+(2*m-1:2*m);
    [Am,Ba,info] = build_resonance_model(modes(m),cfg.limits.Amax);
    A(rows,rows) = Am;
    A(rows,3) = Ba; % oscillator input is physical acceleration state [mm/s^2]
    info.q_index = rows(1);
    info.qdot_index = rows(2);
    modeInfo(m) = info;
end
Md = expm([A B; zeros(1,n+1)]*cfg.Ts);
model.A = A;
model.B = B;
model.Ad = Md(1:n,1:n);
model.Bd = Md(1:n,n+1);
model.n_axis_state = n;
model.n_modes = nMode;
model.mode = modeInfo;
model.motion_indices = 1:6;
model.scale_axis = [cfg.scale.position; cfg.scale.velocity; ...
    cfg.scale.acceleration; cfg.scale.jerk; cfg.scale.snap; ...
    cfg.scale.crackle; repmat([1; 2*pi*10],nMode,1)];
for m = 1:nMode
    model.scale_axis(6+2*m) = modeInfo(m).wn;
end
model.Ad_scaled = diag(1./model.scale_axis)*model.Ad*diag(model.scale_axis);
model.Bd_scaled = diag(1./model.scale_axis)*model.Bd*cfg.scale.pop;
end
