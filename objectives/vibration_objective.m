function [value,components] = vibration_objective(modal,modalVelocity,model,cfg)
%VIBRATION_OBJECTIVE Normalized modal energy plus differentiable peak proxy.
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
value=cfg.objective.vibration_energy_weight*components.energy + ...
    cfg.objective.vibration_peak_weight*components.peak_squared;
end
