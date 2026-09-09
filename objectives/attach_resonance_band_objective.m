function solution = attach_resonance_band_objective(solution,cfg,reference,enforce)
%ATTACH_RESONANCE_BAND_OBJECTIVE Report and validate the spectral objective.
if nargin<4, enforce=false; end
proxy=calculate_resonance_band_proxy(solution,cfg);
frequency=calculate_frequency_metrics(solution,cfg);
actualAxis=[frequency.mode(1).band_energy.modal_x; ...
    frequency.mode(1).band_energy.modal_y];
actualTotal=sum(actualAxis);

if isempty(reference) || reference.proxy<=0 || reference.actual<=0
    normalized=0;
    contribution=0;
    targetProxy=NaN;
    targetActual=NaN;
    pass=true;
else
    normalized=proxy.total/reference.proxy;
    contribution=cfg.objective.resonance_band_weight*normalized;
    targetRatio=1-cfg.objective.resonance_band_min_reduction;
    targetProxy=targetRatio*reference.proxy;
    targetActual=targetRatio*reference.actual;
    tolerance=cfg.validation.resonance_band_relative_tol;
    pass=~enforce || (proxy.total<=targetProxy*(1+tolerance) && ...
        actualTotal<=targetActual*(1+tolerance));
end

if isfield(solution.objectives,'vibration_time_domain')
    timeDomain=solution.objectives.vibration_time_domain;
else
    timeDomain=solution.objectives.vibration;
end
solution.objectives.vibration_time_domain=timeDomain;
solution.objectives.vibration_components.resonance_band_energy=proxy.total;
solution.objectives.vibration_components.resonance_band_energy_axis=proxy.axis;
solution.objectives.vibration_components.resonance_band_normalized=normalized;
solution.objectives.vibration_components.resonance_band_contribution=contribution;
solution.objectives.vibration=timeDomain+contribution;
solution.validation.resonance_band=struct('frequency',proxy.frequency, ...
    'half_bandwidth',proxy.half_bandwidth,'proxy_axis',proxy.axis, ...
    'proxy_total',proxy.total,'actual_axis',actualAxis, ...
    'actual_total',actualTotal,'target_proxy',targetProxy, ...
    'target_actual',targetActual,'required_reduction', ...
    cfg.objective.resonance_band_min_reduction,'enforced',logical(enforce), ...
    'pass',logical(pass));
solution.validation.pass=solution.validation.pass && pass;
if isfield(solution,'feasible'), solution.feasible=solution.feasible && pass; end
end
