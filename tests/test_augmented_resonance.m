function test_augmented_resonance()
cfg = trajectory_config();
model = build_augmented_dynamics(cfg);
M = expm([model.A model.B; zeros(1,model.n_axis_state+1)]*cfg.Ts);
assert(max(abs(M(1:end-1,1:end-1)-model.Ad),[],'all')<1e-14);
assert(max(abs(M(1:end-1,end)-model.Bd))<1e-14);
fprintf('PASS test_augmented_resonance\n');
end
