function [value,components] = vibration_objective(modal,modalVelocity,acceleration,jerk,model,cfg)
%VIBRATION_OBJECTIVE Modal and normalized axis RMS/peak vibration metrics.
% modal arrays are axis-by-mode-by-time. Numeric reporting uses true peaks;
% the OCP replaces the peak with epigraph variables.
value=0; components.energy=0; components.peak_squared=0;
for axis=1:size(modal,1)
    for m=1:model.n_modes
        q=squeeze(modal(axis,m,:)); qd=squeeze(modalVelocity(axis,m,:));
        E=sum(q.^2+(qd/model.mode(m).wn).^2)/numel(q);
        p=max(abs(q));
        components.energy=components.energy+E;
        components.peak_squared=components.peak_squared+p^2;
    end
end
normalizedAcceleration=acceleration/cfg.limits.Amax;
normalizedJerk=jerk/cfg.limits.Jmax;
components.acceleration_rms_axis=sqrt(mean(normalizedAcceleration.^2,2));
components.acceleration_peak_axis=max(abs(normalizedAcceleration),[],2);
components.jerk_rms_axis=sqrt(mean(normalizedJerk.^2,2));
components.jerk_peak_axis=max(abs(normalizedJerk),[],2);
components.acceleration_rms=sum(components.acceleration_rms_axis);
components.acceleration_peak=sum(components.acceleration_peak_axis);
components.jerk_rms=sum(components.jerk_rms_axis);
components.jerk_peak=sum(components.jerk_peak_axis);
value=cfg.objective.vibration_energy_weight*components.energy + ...
    cfg.objective.vibration_peak_weight*components.peak_squared + ...
    cfg.objective.acceleration_rms_weight*components.acceleration_rms + ...
    cfg.objective.acceleration_peak_weight*components.acceleration_peak + ...
    cfg.objective.jerk_rms_weight*components.jerk_rms + ...
    cfg.objective.jerk_peak_weight*components.jerk_peak;
end
