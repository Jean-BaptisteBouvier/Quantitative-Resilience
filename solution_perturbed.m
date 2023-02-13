% Calculates the maximum lambda such that lambda*y = Bu + Cw when w is the
% worst undesirable input and u chosen as a function of w to maximize
% lambda.
% The time to reach the target is time = 1/lambda

function [time, u, w] = solution_perturbed(B_bar, u_min, u_max, d, col_loss)

[n, m] = size(B_bar);
mC = length(col_loss);
mB = m - mC;

% lower and upper bounds
lb_u = u_min;
ub_u = u_max;
lb_minus_lambda = -1e5; % if =-1 then time = 1 for all targets where lambda >= 1
ub_minus_lambda = 0;

% Matrices for linprog
f = [zeros(m, 1);1];
A_eq = [B_bar, d];
b_eq = zeros(n,1);

options = optimoptions('linprog','Display','none');
[u,w] = deal([]);
time = Inf;

solution = -10000*ones(mB+mC+1,1);

% We build W containing all corners of the hypercube of dimension mC
L = nchoosek( [u_max(col_loss)', u_min(col_loss)'], mC);
L = unique(L, 'rows');
W = [];
for i = 1:length(L(:,1))
    W = [W; unique(perms(L(i,:)), 'rows')];
end
W = W';


for w_bound = W
    try
        lb_u(col_loss) = w_bound;
        ub_u(col_loss) = w_bound;
        sol = linprog(f, [], [], A_eq, b_eq, [lb_u; lb_minus_lambda], [ub_u; ub_minus_lambda], options);  
        lambda = -sol(end);
    catch ME
        w = w_bound;
        return
    end

    if lambda < -solution(end) % lambda must be the smallest since we are choosing the worst w
        solution = sol;
    end
end


lambda = -solution(end);
if lambda > 0
    u = solution(1:end-1); u(col_loss) = [];
    w = solution(col_loss);
end

time = abs(1/lambda);

end