function solution = choose_compromise_solution(solutions,data)
%CHOOSE_COMPROMISE_SOLUTION Select closest feasible normalized Pareto knee.
valid=find(data.Feasible & data.LambdaVibration>0);
if isempty(valid), error('POP_MOO:NoParetoSolution','No vibration-aware feasible solution.'); end
jv=data.Jvibration(valid); js=data.Jstraightness(valid);
nv=(jv-min(jv))/max(max(jv)-min(jv),eps);
ns=(js-min(js))/max(max(js)-min(js),eps);
distance=sqrt(nv.^2+ns.^2);
[~,k]=min(distance);
solution=solutions{valid(k)};
solution.compromise.lambda_vibration=data.LambdaVibration(valid(k));
solution.compromise.lambda_straightness=data.LambdaStraightness(valid(k));
solution.compromise.normalized_utopia_distance=distance(k);
end
