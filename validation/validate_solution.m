function report = validate_solution(solution,path,cfg)
%VALIDATE_SOLUTION Independent feasibility audit; solver status is not trusted.
model=solution.model; nx=model.n_axis_state; N=solution.N;
dynamicsResidual=0;
for axis=1:2
    predicted=model.Ad*solution.Xaxis(:,1:N,axis)+model.Bd*solution.U(axis,:);
    residual=(solution.Xaxis(:,2:N+1,axis)-predicted)./model.scale_axis;
    dynamicsResidual=max(dynamicsResidual,max(abs(residual),[],'all'));
end
p0=path.points(1,1:2).';
initialMotion=[solution.position(:,1)-p0;solution.velocity(:,1); ...
    solution.acceleration(:,1);solution.jerk(:,1);solution.snap(:,1);solution.crackle(:,1)];
finalMotion=[solution.position(:,end)-p0;solution.velocity(:,end); ...
    solution.acceleration(:,end);solution.jerk(:,end);solution.snap(:,end);solution.crackle(:,end)];
boundaryResidual=max(abs([initialMotion./[cfg.scale.position*ones(2,1); ...
    cfg.scale.velocity*ones(2,1);cfg.scale.acceleration*ones(2,1); ...
    cfg.scale.jerk*ones(2,1);cfg.scale.snap*ones(2,1);cfg.scale.crackle*ones(2,1)]; ...
    finalMotion./[cfg.scale.position*ones(2,1);cfg.scale.velocity*ones(2,1); ...
    cfg.scale.acceleration*ones(2,1);cfg.scale.jerk*ones(2,1); ...
    cfg.scale.snap*ones(2,1);cfg.scale.crackle*ones(2,1)]]));
endpointPop=max(abs(solution.U(:,[1 end])),[],'all')/cfg.limits.POPMax;
usage=calculate_constraint_usage(solution,cfg);
contour=calculate_contour_error(solution,path,cfg);
corners=calculate_corner_sagitta(solution,path,cfg);

% Re-evaluate the segment-assigned tubes, progress, and tangent velocity.
maxTubeViolation=0; maxTauViolation=0; maxBackward=0; cursor=0;
for i=1:path.n_segments
    Ni=solution.Nvec(i); idx=cursor+(1:Ni+1); seg=path.segment(i);
    dp=solution.position(:,idx)-seg.p0(1:2).';
    tau=seg.tangent(1:2)*dp; normal=seg.normal(1:2)*dp;
    dCorner=min(max(tau,0),max(seg.length-tau,0));
    allowed=dynamic_tolerance(dCorner,cfg);
    maxTubeViolation=max(maxTubeViolation,max(abs(normal)-allowed));
    maxTauViolation=max([maxTauViolation,max(-cfg.geometry.overlap_extension-tau), ...
        max(tau-seg.length-cfg.geometry.overlap_extension)]);
    maxBackward=max([maxBackward,max(-(diff(tau)+cfg.validation.geometry_abs_tol)), ...
        max(-(seg.tangent(1:2)*solution.velocity(:,idx)+cfg.validation.backward_velocity_tol))]);
    cursor=cursor+Ni;
end
sampleResidual=abs(solution.T-solution.N*cfg.Ts);
hasNonfinite=any(~isfinite([solution.Xaxis(:);solution.U(:)]));
tol=cfg.validation;
checks.dynamics=dynamicsResidual<=tol.dynamics_abs_tol;
checks.boundary=boundaryResidual<=tol.boundary_abs_tol;
checks.endpoint_pop=endpointPop<=tol.relative_limit_tol;
checks.kinematics=usage.max_ratio<=1+tol.relative_limit_tol;
checks.overall_geometry=max(contour.error-contour.allowed_error)<=tol.geometry_abs_tol;
checks.line_geometry=contour.line_max<=cfg.geometry.line_tolerance+tol.geometry_abs_tol;
checks.corner_sagitta=max([corners.max_error])<=cfg.geometry.corner_tolerance+tol.geometry_abs_tol;
checks.assigned_tube=maxTubeViolation<=tol.geometry_abs_tol;
checks.segment_bounds=maxTauViolation<=tol.geometry_abs_tol;
checks.segment_sequence=maxBackward<=tol.geometry_abs_tol;
checks.sample_period=sampleResidual<=tol.time_abs_tol;
checks.finite=~hasNonfinite;
report.pass=all(structfun(@(x)logical(x),checks));
report.checks=checks; report.dynamics_residual=dynamicsResidual;
report.boundary_residual=boundaryResidual; report.endpoint_pop_ratio=endpointPop;
report.usage=usage; report.contour=contour; report.corners=corners;
report.max_assigned_tube_violation=maxTubeViolation;
report.max_segment_bound_violation=maxTauViolation;
report.max_backward_violation=maxBackward;
report.sample_period_residual=sampleResidual;
report.max_violation=max([dynamicsResidual,boundaryResidual,endpointPop, ...
    max(0,usage.max_ratio-1),max(0,max(contour.error-contour.allowed_error)), ...
    max(0,maxTubeViolation),max(0,maxTauViolation),max(0,maxBackward)]);
end
