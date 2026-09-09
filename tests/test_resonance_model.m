function test_resonance_model(cfg)
% The configured resonance must respond more strongly at f0 than off-resonance.
if nargin<1, cfg=trajectory_config(); end
model = build_augmented_dynamics(cfg);
f0=cfg.resonance.mode(1).frequency;
Fs=1/cfg.Ts;
assert(f0>0 && f0<Fs/2,'Configured resonance frequency must be below Nyquist.');
if 4*f0<0.45*Fs
    offFrequency=4*f0;
else
    offFrequency=f0/4;
end
N = 5000; t = (0:N-1)*cfg.Ts;
response = zeros(1,2);
for trial = 1:2
    frequencies=[f0,offFrequency];
    f=frequencies(trial);
    % Simulate oscillator exactly with piecewise-constant normalized acceleration.
    rows=[model.mode(1).q_index,model.mode(1).qdot_index];
    A=model.A(rows,rows);
    Ba=model.A(rows,3)*cfg.limits.Amax;
    Md = expm([A Ba; zeros(1,3)]*cfg.Ts);
    x = zeros(2,N+1);
    r = sin(2*pi*f*t);
    for k=1:N, x(:,k+1)=Md(1:2,1:2)*x(:,k)+Md(1:2,3)*r(k); end
    response(trial)=rms(x(1,floor(N/2)+1:end));
end
assert(response(1)>5*response(2), ...
    'Configured oscillator is not selective at %.6g Hz.',f0);
fprintf('PASS test_resonance_model (%.6g/%.6g Hz RMS ratio %.2f)\n', ...
    f0,offFrequency,response(1)/response(2));
end
