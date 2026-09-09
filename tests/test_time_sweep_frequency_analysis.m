function test_time_sweep_frequency_analysis()
%TEST_TIME_SWEEP_FREQUENCY_ANALYSIS Check allocation and FFT diagnostics.
cfg=trajectory_config();
minimum=struct('N',40,'Nvec',[10 10 10 10]);
path=struct('lengths',[20 20 20 20]);
previous=minimum.Nvec;
for targetN=40:52
    allocation=stage_b_integer_allocation(minimum,path,targetN);
    assert(sum(allocation)==targetN);
    assert(all(allocation>=minimum.Nvec));
    assert(all(allocation-previous>=0));
    assert(sum(allocation-previous)<=1);
    previous=allocation;
end

N=999;
time=(0:N)*cfg.Ts;
solution.N=N;
solution.acceleration=[sin(2*pi*50*time);0.5*sin(2*pi*50*time)];
solution.jerk=[cos(2*pi*50*time);0.5*cos(2*pi*50*time)];
solution.modal=zeros(2,1,N+1);
solution.modal(1,1,:)=reshape(sin(2*pi*50*time),1,1,[]);
solution.modal(2,1,:)=reshape(0.5*sin(2*pi*50*time),1,1,[]);
solution.model.n_modes=1;
spectrum=calculate_amplitude_spectrum(solution,cfg);
[~,peakIndex]=max(spectrum.signal.modal_x.amplitude);
peakFrequency=spectrum.signal.modal_x.f(peakIndex);
assert(abs(peakFrequency-50)<0.3);
fprintf('PASS test_time_sweep_frequency_analysis (FFT peak %.3f Hz)\n',peakFrequency);
end
