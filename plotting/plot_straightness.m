function plot_straightness(solution,path,cfg)
vperp=nan(1,solution.N+1); cursor=0;
for i=1:path.n_segments
    idx=cursor+(1:solution.Nvec(i)+1); n=path.segment(i).normal(1:2);
    vperp(idx)=n*solution.velocity(:,idx); cursor=cursor+solution.Nvec(i);
end
fig=figure('Visible',cfg.plot.visible,'Color','w');
plot(solution.time,vperp,'LineWidth',1.0); yline(0,'k:'); grid on;
xlabel('Time [s]'); ylabel('Transverse velocity v_{perp} [mm/s]');
title('Straight-line transverse velocity');
save_publication_figure(fig,fullfile(cfg.output.figures,'11_transverse_velocity.png'),cfg);
end
