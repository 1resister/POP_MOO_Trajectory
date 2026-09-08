function test_high_order_smoothstep()
[~,v]=generate_high_order_smoothstep([0 1]);
assert(abs(v(1,1))<1e-11 && abs(v(1,2)-1)<1e-9);
assert(max(abs(v(2:6,:)),[],'all')<2e-8);
fprintf('PASS test_high_order_smoothstep\n');
end
