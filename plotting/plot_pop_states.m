function plot_pop_states(solution,cfg)
signals={solution.snap,solution.crackle,[solution.U,solution.U(:,end)]};
names={'Snap','Crackle','POP'}; units={'mm/s^4','mm/s^5','mm/s^6'};
files={'08_snap','09_crackle','10_pop'};
limits=[cfg.limits.SnapMax,cfg.limits.CrackleMax,cfg.limits.POPMax];
for i=1:3
    fig=figure('Visible',cfg.plot.visible,'Color','w');
    plot(solution.time,signals{i}(1,:),'LineWidth',0.9); hold on;
    plot(solution.time,signals{i}(2,:),'LineWidth',0.9);
    yline(limits(i),'k--'); yline(-limits(i),'k--'); grid on;
    xlabel('Time [s]'); ylabel(sprintf('%s [%s]',names{i},units{i}));
    title(sprintf('Axis %s',lower(names{i}))); legend('X','Y','Limits','Location','best');
    save_publication_figure(fig,fullfile(cfg.output.figures,[files{i},'.png']),cfg);
end
end
