function frequency = calculate_frequency_metrics(solution,cfg)
%CALCULATE_FREQUENCY_METRICS PSD and resonance-band integrals at Fs=1/Ts.
Fs=1/cfg.Ts;
signals.ax=solution.acceleration(1,:).'; signals.ay=solution.acceleration(2,:).';
signals.jx=solution.jerk(1,:).'; signals.jy=solution.jerk(2,:).';
if solution.model.n_modes>0
    signals.modal_x=squeeze(solution.modal(1,1,:));
    signals.modal_y=squeeze(solution.modal(2,1,:));
else
    signals.modal_x=zeros(solution.N+1,1); signals.modal_y=signals.modal_x;
end
names=fieldnames(signals); frequency.Fs=Fs;
for i=1:numel(names)
    x=signals.(names{i}); x=x(:)-mean(x);
    [Pxx,f,method]=local_psd(x,Fs);
    frequency.signal.(names{i}).f=f;
    frequency.signal.(names{i}).psd=Pxx;
    frequency.signal.(names{i}).method=method;
end
for m=1:solution.model.n_modes
    f0=solution.model.mode(m).frequency; bw=cfg.frequency.bandwidth;
    frequency.mode(m).frequency=f0;
    for name={'ax','ay','jx','jy','modal_x','modal_y'}
        key=name{1}; spec=frequency.signal.(key);
        band=spec.f>=f0-bw & spec.f<=f0+bw;
        if nnz(band)>=2
            energy=trapz(spec.f(band),spec.psd(band));
        else
            [~,nearest]=min(abs(spec.f-f0)); energy=spec.psd(nearest)*(2*bw);
        end
        frequency.mode(m).band_energy.(key)=energy;
    end
end
end

function [Pxx,f,method]=local_psd(x,Fs)
n=numel(x); nfft=max(4096,2^nextpow2(n));
if exist('pwelch','file')==2
    windowLength=min(n,1024);
    if windowLength<16, windowLength=n; end
    window=hamming(windowLength,'periodic'); overlap=floor(windowLength/2);
    [Pxx,f]=pwelch(x,window,overlap,nfft,Fs,'onesided'); method='pwelch';
else
    X=fft(x,nfft); Pxx=abs(X(1:nfft/2+1)).^2/(Fs*n);
    if numel(Pxx)>2, Pxx(2:end-1)=2*Pxx(2:end-1); end
    f=(0:nfft/2)'*Fs/nfft; method='fft_fallback';
end
end
