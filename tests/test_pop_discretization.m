function test_pop_discretization()
%TEST_POP_DISCRETIZATION Matrix exponential equals the exact polynomial map.
cfg = trajectory_config();
[Ad,Bd,v] = exact_pop_discretization(cfg.Ts);
assert(v.max_abs_error < 50*eps, 'Analytic and expm ZOH maps differ.');
x = [1.2;-3.4;50;-800;2e4;-5e6];
u = 1.1e11;
h = cfg.Ts;
expected = [x(1)+x(2)*h+x(3)*h^2/2+x(4)*h^3/6+x(5)*h^4/24+x(6)*h^5/120+u*h^6/720;
    x(2)+x(3)*h+x(4)*h^2/2+x(5)*h^3/6+x(6)*h^4/24+u*h^5/120;
    x(3)+x(4)*h+x(5)*h^2/2+x(6)*h^3/6+u*h^4/24;
    x(4)+x(5)*h+x(6)*h^2/2+u*h^3/6;
    x(5)+x(6)*h+u*h^2/2;
    x(6)+u*h];
actual = Ad*x+Bd*u;
assert(max(abs(actual-expected)) < 2e-10, 'POP exact integration formula failed.');
fprintf('PASS test_pop_discretization (max matrix error %.3g)\n',v.max_abs_error);
end
