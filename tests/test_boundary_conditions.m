function test_boundary_conditions()
cfg=trajectory_config(); path=generate_square_path(cfg);
[s,~]=find_initial_feasible_duration(path,cfg);
motion=s.Xaxis(1:6,:,:);
initial=squeeze(motion(:,1,:)); final=squeeze(motion(:,end,:));
assert(max(abs(initial(2:6,:)),[],'all')<1e-9);
assert(max(abs(final(2:6,:)),[],'all')<2e-4);
assert(norm(initial(1,:)-path.points(1,1:2).',inf)<1e-9);
assert(norm(final(1,:)-path.points(end,1:2).',inf)<2e-7);
assert(max(abs(s.U(:,[1 end])),[],'all')<1e-9);
% Every intermediate segment endpoint is stationary in this seed.
c=cumsum(s.Nvec);
assert(max(abs(s.velocity(:,c(1:end-1)+1)),[],'all')<2e-6);
fprintf('PASS test_boundary_conditions (seed N=%d, T=%.3f s)\n',s.N,s.T);
end
