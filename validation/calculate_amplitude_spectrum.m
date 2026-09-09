function spectrum = calculate_amplitude_spectrum(solution,cfg)
%CALCULATE_AMPLITUDE_SPECTRUM Windowed one-sided FFT amplitude spectra.
Fs=1/cfg.Ts;
signals.ax=solution.acceleration(1,:).';
signals.ay=solution.acceleration(2,:).';
signals.jx=solution.jerk(1,:).';
signals.jy=solution.jerk(2,:).';
if solution.model.n_modes>0
    signals.modal_x=squeeze(solution.modal(1,1,:));
    signals.modal_y=squeeze(solution.modal(2,1,:));
else
    signals.modal_x=zeros(solution.N+1,1);
    signals.modal_y=signals.modal_x;
end

names=fieldnames(signals);
spectrum.Fs=Fs;
for index=1:numel(names)
    x=signals.(names{index});
    x=x(:)-mean(x);
    n=numel(x);
    nfft=max(4096,2^nextpow2(n));
    window=0.54-0.46*cos(2*pi*(0:n-1)'/n);
    transform=fft(x.*window,nfft);
    amplitude=abs(transform(1:floor(nfft/2)+1))/sum(window);
    if numel(amplitude)>2
        amplitude(2:end-1)=2*amplitude(2:end-1);
    end
    spectrum.signal.(names{index}).f=(0:floor(nfft/2))'*Fs/nfft;
    spectrum.signal.(names{index}).amplitude=amplitude;
end
end
