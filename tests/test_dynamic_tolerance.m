function test_dynamic_tolerance()
cfg = trajectory_config();
d = [0 1 2 3];
e = dynamic_tolerance(d,cfg);
assert(abs(e(1)-0.10)<eps && abs(e(3)-0.05)<eps && abs(e(4)-0.05)<eps);
left = dynamic_tolerance(2-1e-7,cfg);
right = dynamic_tolerance(2+1e-7,cfg);
assert(abs(left-right)<1e-10, 'Tolerance transition is discontinuous.');
fprintf('PASS test_dynamic_tolerance\n');
end
