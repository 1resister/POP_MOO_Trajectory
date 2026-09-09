function projection = resonance_band_projection(N,Ts,frequency,halfBandwidth)
%RESONANCE_BAND_PROJECTION Linear coefficients matching the Welch band metric.
% For the current near-time square case N+1<1024, pwelch uses one periodic
% Hamming window.  The returned projections reproduce its mean-removed,
% one-sided PSD integral exactly while remaining suitable for an NLP.
sampleCount=N+1;
Fs=1/Ts;
nfft=max(4096,2^nextpow2(sampleCount));
time=(0:sampleCount-1)'/Fs;
window=0.54-0.46*cos(2*pi*(0:sampleCount-1)'/sampleCount);
allBins=(0:floor(nfft/2)).';
allFrequencies=allBins*Fs/nfft;
selected=allFrequencies>=frequency-halfBandwidth & ...
    allFrequencies<=frequency+halfBandwidth;
if ~any(selected)
    [~,nearest]=min(abs(allFrequencies-frequency));
    selected(nearest)=true;
end
bins=allBins(selected);
frequencies=allFrequencies(selected);

if numel(frequencies)>=2
    integrationWeights=ones(numel(frequencies),1)*(Fs/nfft);
    integrationWeights([1,end])=integrationWeights([1,end])/2;
else
    integrationWeights=2*halfBandwidth;
end
oneSided=2*ones(numel(bins),1);
oneSided(bins==0)=1;
if rem(nfft,2)==0, oneSided(bins==nfft/2)=1; end
psdWeights=integrationWeights.*oneSided/(Fs*sum(window.^2));

cosine=zeros(numel(frequencies),sampleCount);
sine=zeros(numel(frequencies),sampleCount);
for k=1:numel(frequencies)
    cosineBasis=window.*cos(2*pi*frequencies(k)*time);
    sineBasis=window.*sin(2*pi*frequencies(k)*time);
    cosine(k,:)=cosineBasis-mean(cosineBasis);
    sine(k,:)=sineBasis-mean(sineBasis);
end
projection.frequency=frequency;
projection.half_bandwidth=halfBandwidth;
projection.frequencies=frequencies.';
projection.cosine=cosine;
projection.sine=sine;
projection.weights=psdWeights.';
projection.nfft=nfft;
projection.sample_count=sampleCount;
end
