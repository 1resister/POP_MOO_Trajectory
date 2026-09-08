function test_resonance_model()
% A 10 Hz input must excite the configured 10 Hz mode more than 40 Hz.
cfg = trajectory_config();
model = build_augmented_dynamics(cfg);
N = 5000; t = (0:N-1)*cfg.Ts;
response = zeros(1,2);
for trial = 1:2
    f = [10 40]; f = f(trial);
    % Simulate oscillator exactly with piecewise-constant normalized acceleration.
    A = model.A(7:8,7:8);
    Ba = model.A(7:8,3)*cfg.limits.Amax;
    Md = expm([A Ba; zeros(1,3)]*cfg.Ts);
    x = zeros(2,N+1);
    r = sin(2*pi*f*t);
    for k=1:N, x(:,k+1)=Md(1:2,1:2)*x(:,k)+Md(1:2,3)*r(k); end
    response(trial)=rms(x(1,2001:end));
end
assert(response(1)>5*response(2),'Configured oscillator is not selective at 10 Hz.');
fprintf('PASS test_resonance_model (10/40 Hz RMS ratio %.2f)\n',response(1)/response(2));
end
