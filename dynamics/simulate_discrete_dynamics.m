function X = simulate_discrete_dynamics(Ad, Bd, x0, U)
%SIMULATE_DISCRETE_DYNAMICS Propagate an exact discrete LTI model.
% U is nu-by-N; X is nx-by-(N+1).
N = size(U,2);
X = zeros(numel(x0),N+1);
X(:,1) = x0(:);
for k = 1:N
    X(:,k+1) = Ad*X(:,k)+Bd*U(:,k);
end
end
