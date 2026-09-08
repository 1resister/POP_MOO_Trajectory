function plot_axis_kinematics(solution,cfg)
signals={solution.velocity,solution.acceleration,solution.jerk};
names={'Velocity','Acceleration','Jerk'};
units={'mm/s','mm/s^2','mm/s^3'}; files={'05_velocity','06_acceleration','07_jerk'};
limits=[cfg.limits.Vmax,cfg.limits.Amax,cfg.limits.Jmax];
for i=1:3
    fig=figure('Visible',cfg.plot.visible,'Color','w');
    plot(solution.time,signals{i}(1,:),'LineWidth',1.0); hold on;
    plot(solution.time,signals{i}(2,:),'LineWidth',1.0);
    yline(limits(i),'k--'); yline(-limits(i),'k--'); grid on;
    xlabel('Time [s]'); ylabel(sprintf('%s [%s]',names{i},units{i}));
    title(sprintf('Axis %s',lower(names{i}))); legend('X','Y','Limits','Location','best');
    save_publication_figure(fig,fullfile(cfg.output.figures,[files{i},'.png']),cfg);
end
end
