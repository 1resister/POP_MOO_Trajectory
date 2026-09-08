function usage = calculate_constraint_usage(solution,cfg)
%CALCULATE_CONSTRAINT_USAGE Independently recompute per-axis peak ratios.
names={'Velocity','Acceleration','Jerk','Snap','Crackle','POP'};
limits=[cfg.limits.Vmax,cfg.limits.Amax,cfg.limits.Jmax, ...
    cfg.limits.SnapMax,cfg.limits.CrackleMax,cfg.limits.POPMax];
signals={solution.velocity,solution.acceleration,solution.jerk, ...
    solution.snap,solution.crackle,solution.U};
ratio=zeros(6,2); peak=zeros(6,2);
for i=1:6
    peak(i,:)=max(abs(signals{i}),[],2).';
    ratio(i,:)=peak(i,:)/limits(i);
end
usage.names=names; usage.limits=limits; usage.peak=peak; usage.ratio=ratio;
usage.max_ratio=max(ratio,[],'all');
usage.table=table(string(names(:)),peak(:,1),peak(:,2),ratio(:,1),ratio(:,2), ...
    'VariableNames',{'Quantity','PeakX','PeakY','UtilisationX','UtilisationY'});
end
