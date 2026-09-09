function metrics = calculate_resonance_band_proxy(solution,cfg)
%CALCULATE_RESONANCE_BAND_PROXY Exact numeric value of the OCP band metric.
metrics.frequency=solution.model.mode(1).frequency;
metrics.half_bandwidth=cfg.frequency.bandwidth;
metrics.axis=zeros(2,1);
metrics.total=0;
if solution.model.n_modes==0, return; end
projection=resonance_band_projection(solution.N,cfg.Ts, ...
    metrics.frequency,metrics.half_bandwidth);
for axisIndex=1:2
    q=squeeze(solution.modal(axisIndex,1,:)).';
    realPart=q*projection.cosine.';
    imaginaryPart=q*projection.sine.';
    metrics.axis(axisIndex)=sum(projection.weights.* ...
        (realPart.^2+imaginaryPart.^2));
end
metrics.total=sum(metrics.axis);
metrics.projection=projection;
end
