function [coeff, values] = generate_high_order_smoothstep(tau)
%GENERATE_HIGH_ORDER_SMOOTHSTEP Degree-11 endpoint-flat polynomial.
% q(0)=0, q(1)=1 and d^r q/dtau^r=0 at both ends, r=1,...,5.
% Coefficients are solved from the 12 boundary equations, not hard-coded.
arguments
    tau double = linspace(0,1,101)
end
n = 11;
A = zeros(12,n+1); b = zeros(12,1);
row = 1;
for endpoint = [0 1]
    for derivative = 0:5
        for power = derivative:n
            A(row,power+1) = factorial(power)/factorial(power-derivative) * ...
                endpoint^(power-derivative);
        end
        if endpoint==1 && derivative==0, b(row)=1; end
        row = row+1;
    end
end
coeff = A\b; % ascending powers: coeff(power+1)*tau^power
values = zeros(7,numel(tau));
for derivative = 0:6
    for power = derivative:n
        values(derivative+1,:) = values(derivative+1,:) + ...
            coeff(power+1)*factorial(power)/factorial(power-derivative) .* ...
            tau(:).'.^(power-derivative);
    end
end
end
