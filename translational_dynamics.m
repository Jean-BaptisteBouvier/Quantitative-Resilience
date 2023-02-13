%%% Quantitative resilience for UAV, non symmetric input sets

clc
clear variables

%%% Translational matrix
psi = 45*pi/180; % [rad] yaw angle of the UAV (heading)
[B_t, ~] = Octorotor(psi); % theta and phi = 0, pitch and roll angle must be null
B_bar = B_t; % resilience unchanged by the value of psi

[n,m] = size(B_bar);


% Constraints
mass = 1.64; % [kg] mass
g = 9.81; % [m/s^2] gravity
u_bar_min = -mass*g/4*[1; 1; 1; 1; 0; 0; 0; 0]; % lower bound for u_bar
k = 1e-5; % thrust coefficient
omega_max = 2*pi*8000/60; % 8000 rpm into rad/s
u_bar_max = k*omega_max^2 + u_bar_min; % upper bound for u_bar

r_q = zeros(1,m);
time_ratio = zeros(2,m);
ratio = zeros(2,m);


d = [1;0;0]; % direction of motion if not C for the worst case

for col_loss = 1:m % between 1 and m
    
    C = B_bar(:,col_loss);
    B = B_bar; B(:,col_loss) = [];
    w_min = u_bar_min(col_loss); % lower bound for w
    u_min = u_bar_min; u_min(col_loss) = []; % lower bound for u
    w_max = u_bar_max(col_loss); % upper bound for w
    u_max = u_bar_max; u_max(col_loss) = []; % upper bound for u

%     d = C;
    [time_unperturbed, u_bar_p] = solution_unperturbed(B_bar, u_bar_min, u_bar_max, d);
    [time_perturbed, u_p, w_p] = solution_perturbed(B_bar, u_bar_min, u_bar_max, d, col_loss);
    if time_unperturbed == Inf
        time_ratio(1,col_loss) = Inf;
    else
        time_ratio(1,col_loss) = time_perturbed/time_unperturbed;
    end
    [time_unperturbed, u_bar_m] = solution_unperturbed(B_bar, u_bar_min, u_bar_max, -d);
    [time_perturbed, u_m, w_m] = solution_perturbed(B_bar, u_bar_min, u_bar_max, -d, col_loss);
    if time_unperturbed == Inf
        time_ratio(2,col_loss) = Inf;
    else
        time_ratio(2,col_loss) = time_perturbed/time_unperturbed;
    end
    r_q(col_loss) = 1/max(time_ratio(:,col_loss));
    
    [inv_lambda_plus, u_plus] = solution_unperturbed(B, u_min, u_max, C);
    lambda_plus = 1/inv_lambda_plus;

    ratio(1, col_loss) = (lambda_plus + w_min)/(lambda_plus + w_max);
    
    
    [inv_lambda_moins, u_moins] = solution_unperturbed(B, u_min, u_max, -C);
    lambda_moins = -1/inv_lambda_moins;

    ratio(2, col_loss) = (lambda_moins + w_max)/(lambda_moins + w_min);
end

time_ratio
r_q
ratio
1./ratio
sqrt(r_q)