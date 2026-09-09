function test_vibration_objective_jerk()
cfg=trajectory_config();
model.n_modes=1;
model.mode(1).wn=2*pi*10;
modal=zeros(2,1,3);
modalVelocity=zeros(2,1,3);
modal(1,1,:)=reshape([1,2,-1],1,1,[]);
modal(2,1,:)=reshape([0,0.5,0],1,1,[]);
modalVelocity(1,1,:)=model.mode(1).wn*reshape([0,1,-1],1,1,[]);
modalVelocity(2,1,:)=model.mode(1).wn*reshape([0.25,0,-0.25],1,1,[]);
acceleration=cfg.limits.Amax*[0,0.5,-1;0.25,0,0.25];
jerk=cfg.limits.Jmax*[0,1,-1;0.5,0,0.5];
[actual,c]=vibration_objective(modal,modalVelocity,acceleration,jerk,model,cfg);
expectedEnergy=8/3+0.125;
expectedModalPeak=4.25;
expectedAccelerationRMS=sqrt(1.25/3)+sqrt(0.125/3);
expectedAccelerationPeak=1.25;
expectedJerkRMS=sqrt(2/3)+sqrt(0.5/3);
expectedJerkPeak=1.5;
expected=cfg.objective.vibration_energy_weight*expectedEnergy + ...
    cfg.objective.vibration_peak_weight*expectedModalPeak + ...
    cfg.objective.acceleration_rms_weight*expectedAccelerationRMS + ...
    cfg.objective.acceleration_peak_weight*expectedAccelerationPeak + ...
    cfg.objective.jerk_rms_weight*expectedJerkRMS + ...
    cfg.objective.jerk_peak_weight*expectedJerkPeak;
assert(abs(c.energy-expectedEnergy)<1e-12);
assert(abs(c.peak_squared-expectedModalPeak)<1e-12);
assert(abs(c.acceleration_rms-expectedAccelerationRMS)<1e-12);
assert(abs(c.acceleration_peak-expectedAccelerationPeak)<1e-12);
assert(abs(c.jerk_rms-expectedJerkRMS)<1e-12);
assert(abs(c.jerk_peak-expectedJerkPeak)<1e-12);
assert(norm(c.acceleration_rms_axis-[sqrt(1.25/3);sqrt(0.125/3)])<1e-12);
assert(norm(c.acceleration_peak_axis-[1;0.25])<1e-12);
assert(norm(c.jerk_rms_axis-[sqrt(2/3);sqrt(0.5/3)])<1e-12);
assert(norm(c.jerk_peak_axis-[1;0.5])<1e-12);
assert(abs(actual-expected)<1e-12);
fprintf('PASS test_vibration_objective_axis_rms_peak (J=%.6f)\n',actual);
end
