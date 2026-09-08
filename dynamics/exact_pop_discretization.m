function [Ad, Bd, verification] = exact_pop_discretization(Ts)
%EXACT_POP_DISCRETIZATION Exact ZOH POP chain discretisation.
% Uses both the augmented matrix exponential and analytic factorial terms.
[A,B] = build_pop_chain();
M = expm([A B; zeros(1,7)]*Ts);
Ad = M(1:6,1:6);
Bd = M(1:6,7);

AdAnalytic = zeros(6);
for row = 1:6
    for col = row:6
        order = col-row;
        AdAnalytic(row,col) = Ts^order/factorial(order);
    end
end
BdAnalytic = zeros(6,1);
for row = 1:6
    order = 7-row;
    BdAnalytic(row) = Ts^order/factorial(order);
end
verification.Ad_analytic = AdAnalytic;
verification.Bd_analytic = BdAnalytic;
verification.max_abs_error = max(abs([Ad(:)-AdAnalytic(:); Bd-BdAnalytic]));
end
