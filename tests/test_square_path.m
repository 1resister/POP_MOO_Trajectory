function test_square_path()
cfg = trajectory_config();
p = generate_square_path(cfg);
assert(p.n_segments==4 && abs(p.total_length-80)<1e-12);
assert(norm(p.points(1,:)-p.points(end,:))<eps);
assert(all(abs(p.lengths-20)<eps));
assert(all(abs(vecnorm(reshape([p.segment.tangent],3,[]),2,1)-1)<eps));
fprintf('PASS test_square_path\n');
end
