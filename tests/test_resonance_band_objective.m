function test_resonance_band_objective()
cfg=trajectory_config();
cfg.resonance.mode(1).frequency=50;
model=build_augmented_dynamics(cfg);
N=587; t=(0:N)*cfg.Ts;
solution.N=N; solution.model=model;
solution.modal=zeros(2,model.n_modes,N+1);
solution.modal(1,1,:)=reshape(sin(2*pi*50*t)+0.2*cos(2*pi*47*t),1,1,[]);
solution.modal(2,1,:)=reshape(0.7*sin(2*pi*50*t+0.4),1,1,[]);
solution.acceleration=zeros(2,N+1);
solution.jerk=zeros(2,N+1);

proxy=calculate_resonance_band_proxy(solution,cfg);
frequency=calculate_frequency_metrics(solution,cfg);
actual=frequency.mode(1).band_energy.modal_x+ ...
    frequency.mode(1).band_energy.modal_y;
assert(proxy.total>0 && actual>0);
if exist('pwelch','file')==2
    relativeError=abs(proxy.total-actual)/actual;
    assert(relativeError<1e-10, ...
        'Symbolic band metric does not match Welch PSD integration.');
else
    relativeError=NaN;
end

path=generate_square_path(cfg);
spec=struct('stage','secondary','lambda_vibration',1, ...
    'lambda_straightness',0,'straightness_scale',1,'vibration_scale',1, ...
    'resonance_band_reference',proxy.total, ...
    'resonance_band_reference_actual',actual);
problem=build_pop_ocp(path,cfg,[12,12,12,12],spec);
assert(~isempty(problem.bandCoefficients));
assert(numel(problem.bandProjection.frequencies)>0);
fprintf('PASS test_resonance_band_objective (Welch relative error %.3g)\n', ...
    relativeError);
end
