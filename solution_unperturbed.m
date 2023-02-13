% Calculates the maximum lambda such that lambda*y = B_bar*u with u
% respecting the constraints -u_max, +u_max.
% The time to reach the target is time = 1/lambda

function [time, u] = solution_unperturbed(B_bar, u_min, u_max, d)

[n, m] = size(B_bar);

% lower and upper bounds
lb_u = u_min;
ub_u = u_max;
lb_minus_lambda = -1e5; % if =-1 then time = 1 for all targets where lambda >= 1
ub_minus_lambda = 0;

% Matrices for linprog
f = [zeros(m,1);1];
A_eq = [B_bar, d];
b_eq = zeros(n,1);

options = optimoptions('linprog','Display','none');

u = zeros(m,1);

try
    sol = linprog(f, [], [], A_eq, b_eq, [lb_u; lb_minus_lambda], [ub_u; ub_minus_lambda], options);  
    lambda = -sol(end);
catch ME
    lambda = 0;
end

if lambda > 0
    u = sol(1:m);
end

time = 1/lambda;

end