function value = regularization_objective(Xbar,Ubar,cfg,nAxisState)
%REGULARIZATION_OBJECTIVE Dimensionless weak Stage-A tie-breaker.
snapRows=[5,nAxisState+5]; crackleRows=[6,nAxisState+6];
value=cfg.objective.feas_pop_weight*sum(Ubar(:).^2)/numel(Ubar) + ...
    cfg.objective.feas_crackle_weight*sum(sum(Xbar(crackleRows,:).^2))/size(Xbar,2) + ...
    cfg.objective.feas_snap_weight*sum(sum(Xbar(snapRows,:).^2))/size(Xbar,2);
end
