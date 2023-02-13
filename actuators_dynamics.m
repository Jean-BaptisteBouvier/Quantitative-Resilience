%%% Quantitative Resilience of Generalized Integrators
%%% TAC

clear variables
clc


%%% Translational matrix of the octocopter
psi = 45*pi/180; % [rad] yaw angle of the UAV (heading)
[B_bar, ~] = Octorotor(psi); % theta and phi = 0, pitch and roll angle must be null
[n, m] = size(B_bar);

mass = 1.64; % [kg] mass
g = 9.81; % [m/s^2] gravity
u_bar_min = -mass*g/4*[1; 1; 1; 1; 0; 0; 0; 0]; % lower bound for u_bar
k = 1e-5; % thrust coefficient
omega_max = 2*pi*8000/60; % 8000 rpm into rad/s
u_bar_max = k*omega_max^2 + u_bar_min; % upper bound for u_bar
tau = 0.1;

%%% Calculate T_N
T = 0:0.002:3;
t1 = 1;
t2 = Inf;
% u_bar = @(t) (t > t1).*(t < t2).*u_bar_max + (t <= t1)*u_bar_min + (t >= t2)*u_bar_min;
u_bar = @(t) (t > t1).*(t < t2).*u_bar_max + (t <= t1)*0 + (t >= t2)*u_bar_min;

% whole simulation with actuator dynamics, position and velocity
A_tot = [zeros(n,n), eye(n), zeros(n,m); zeros(n,2*n), B_bar; zeros(m,2*n), -eye(m)/tau];
B_bar_tot = [zeros(2*n,m); eye(m)/tau];
[~, X_N_exp] = ode45(@(t,x) A_tot*x + B_bar_tot*u_bar(t), T, [zeros(2*n,1); u_bar(0)]);
% [~, X_N_exp] = ode45(@(t,x) A_tot*x + B_bar_tot*u_bar(t,x, u_bar_min, u_bar_max), T, [zeros(2*n,1); u_bar(0)]);

% whole simulation with bang-bang actuators, position and velocity
A_bang = [zeros(n,n), eye(n); zeros(n,2*n)];
B_bar_bang = [zeros(n,m); B_bar];
[~, X_N_bang] = ode45(@(t,x) A_bang*x + B_bar_bang*u_bar(t), T, zeros(2*n,1));


%%% Calculate T_M
failure = 1;
uw_minmax = u_bar_min; uw_minmax(failure) = u_bar_max(failure);
uw_maxmin = u_bar_max; uw_maxmin(failure) = u_bar_min(failure);
uw = @(t) (t > t1).*(t < t2).*uw_maxmin + (t <= t1)*0 + (t >= t2)*uw_minmax;

% whole simulation with actuator dynamics, position and velocity
[~, X_M_exp] = ode45(@(t,x) A_tot*x + B_bar_tot*uw(t), T, [zeros(2*n,1); uw(0)]);
[~, X_M_bang] = ode45(@(t,x) A_bang*x + B_bar_bang*uw(t), T, zeros(2*n,1));

%%% Resilience
d = 1;
T_N_bang = T(find(X_N_bang(:,3) > d, 1, 'first'));
T_N_exp = T(find(X_N_exp(:,3) > d, 1, 'first'));
T_M_bang = T(find(X_M_bang(:,3) > d, 1, 'first'));
T_M_exp = T(find(X_M_exp(:,3) > d, 1, 'first'));
r_q_exp = T_N_exp/T_M_exp;
r_q_bang = T_N_bang/T_M_bang;


%%% Plots
N = length(T);
bar_c1 = zeros(1, N);
w_c1 = zeros(1,N);
for i = 1:N
    a = u_bar(T(i));
    bar_c1(i) = a(1);
    a = uw(T(i));
    w_c1(i) = a(1);
end

figure
hold on
grid on
plot(T, bar_c1, 'LineWidth', 2)
plot(T, X_N_exp(:,2*n+1), 'LineWidth', 2)
% [~, omega_1] = ode45(@(t,x) (omega_max - x)/0.1, T, 0);
% plot(T, k*omega_1.^2 - mass*g/4, 'LineWidth', 2)
plot(T, w_c1, 'LineWidth', 2)
plot(T, X_M_exp(:,2*n+failure), 'LineWidth', 2)
legend('$\bar{c}_1$', '$\bar{u}_1$', '$w$ bang', '$w$ smooth','interpreter','latex')
xlabel('time (s)')
ylabel('first input (N)')
% ylabel('first input $k\omega_1^2 - mg/4$ (N)','interpreter','latex')
set(gca,'fontsize', 18);

figure
hold on
grid on
plot(T, X_N_bang(:,n+3), 'LineWidth', 2)
plot(T, X_N_exp(:,n+3), 'LineWidth', 2)
plot(T, X_M_bang(:,n+3), 'LineWidth', 2)
plot(T, X_M_exp(:,n+3), 'LineWidth', 2)
legend('$\dot z_N$ bang', '$\dot z_N$ smooth', '$\dot z_M$ bang', '$\dot z_M$ smooth','interpreter','latex')
xlabel('time (s)')
ylabel('vertical velocity (m/s)')
set(gca,'fontsize', 18);

figure
hold on
grid on
plot(T, X_N_bang(:,3), 'LineWidth', 2)
plot(T, X_N_exp(:,3), 'LineWidth', 2)
plot(T, X_M_bang(:,3), 'LineWidth', 2)
plot(T, X_M_exp(:,3), 'LineWidth', 2)
legend('$z_N$ bang', '$z_N$ smooth', '$z_M$ bang', '$z_M$ smooth','interpreter','latex')
xlabel('time (s)')
ylabel('vertical position (m)')
set(gca,'fontsize', 18);


%%%%%%%%%%%%%%%%%%%%%% functions %%%%%%%%%%%%%%%%%%%%%

% function u = u_bar(t,x, u_bar_min, u_bar_max)
% 
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ADMIRE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %%% Matching matrices with ADMIRE_2
% load('Trim_M0p22ALT30_LinDATA.mat');
% A = Abare;
% A(1,:) = [];
% A(:,1) = [];
% A = A(1:5,1:5);
% B = Bbare;
% B = B(2:6, 1:7);
% B(:,1) = B(:,1) + B(:,2);
% B(:,2) = [];
% B(:,2) = B(:,2) + B(:,3);
% B(:,3) = [];
% B(:,3) = B(:,3) + B(:,4);
% B(:,4) = [];
% 
% %%% ADMIRE matrices from ADMIRE_2
% 
% A = [-0.5432, 0.0137, 0, 0.9778, 0;
%      0, -0.1179, 0.2215, 0, -0.9661;
%      0, -10.5128, -0.9967, 0, 0.6176;
%      2.6221, -0.30, 0, -0.5057, 0;
%      0, 0.7075, -0.0939, 0, -0.2127];
% eig(A)
% norm(A)
% A_sq = A*A;
% eig(A_sq)
% norm(A_sq)
% 
% B = [0.69, -0.0866, -0.0866, 0.04;
%      0, 0.0119, -0.0119, 0.0287;
%      0, -4.2423, 4.2423, 1.4871;
%      1.6532, -1.2735, -1.2735, 0.24;
%      0, -0.2805, 0.2805, -0.8823];
% B2 = A*B-B*20*eye(4);
% B3 = B*20*eye(4);




%%% Driftless ADMIRE from old TAC

% STATES:
% alpha  2 angle of attack (rad)
% beta   3 sideslip angle (rad)
% p      4 body axis roll rate (rad/s)
% q      5 body axis pitch rate (rad/s)
% r      6 body axis yaw rate (rad/s)
% 
% CONTROLS:
%  1 Right Canard, rc (rad)
%  2 Left Canard, lc (rad)
%  3 Right Outboard Elevon, roe (rad)
%  4 Right Inboard Elevon, rie (rad)
%  5 Left Outboard Elevon, lie (rad)
%  6 Left Outboard Elevon, loe (rad)
%  7 Rudder, rud (rad)
%  8 Leading edge flaps (rad)

% Mach = 0.75;
% Alt = 30;
% A = [-1.85, 0, 0, 0.96, 0; 0, -0.41, 0.03, 0, -0.99; 0, -45.84, -3.67, 0, 0.95; 12.73, 0, 0, -2.26, 0; 0, 4.52, -0.19, 0, -0.65];
% B_bar = [-0.01, -0.01, -0.1, -0.19, -0.19, -0.1, 0, 0.01;
%          -0.01, 0.01, 0, 0.03, -0.03, 0, 0.110, 0;
%          1.75, -1.75, -25.2, -22.57, 22.57, 25.2, 11.85, 0;
%          7.07, 7.07, -7.65, -13.12, -13.12, -7.65, 0.03, -0.54;
%          -1.86, 1.86, -1.06, -3.04, 3.04, 1.06, -10.79, 0];

% Mach = 0.8;
% Alt = 3;
% A = [-2.93,0,0,0.950,0;0,-0.580,0.02,0,-0.990;0,-67.960,-5.17,0,1.39;19.770,0,0,-3.52,0;0,5.93,-0.260,0,-0.9];
% B_bar = [-0.01,-0.01,-0.110,-0.250,-0.250,-0.110,0,0.01;-0.02,0.02,0,0.04,-0.04,0,0.140,0;2.12,-2.12,-33.930,-30.730,30.730,33.930,16.710,0;10.660,10.660,-11.020,-20.080,-20.080,-11.020,0.05,-0.620;-2.79,2.79,-1.23,-4.30,4.30,1.23,-15.290,0];
%      
% [n,m] = size(B_bar);
% U_bar = [-55*pi/180, 25*pi/180;  % right canard
%          -55*pi/180, 25*pi/180;  % left canard
%          -25*pi/180, 25*pi/180;  % right outboard elevon
%          -25*pi/180, 25*pi/180;  % right inboard elevon
%          -25*pi/180, 25*pi/180;  % left inboard elevon
%          -25*pi/180, 25*pi/180;  % left outboard elevon
%          -30*pi/180, 30*pi/180;  % rudder
%          -10*pi/180, 30*pi/180]; % leading edge flap

% %%% Resilience verification using zonotopes
% for j = 1:m
%     B_bar(:,j) = B_bar(:,j)*U_bar(j,2);
% end
% for failure = 1:m % id of the actuator lost
% 
%     B = B_bar; B(:, failure) = [];
%     C = B_bar(:, failure);
%     BU = zonotope(zeros(n,1), B);
%     CW = zonotope(zeros(n,1), C);
%     if ~in(BU, CW)
%         warning('The jet is not resilient to the loss of actuator %i.', failure)
%     end
% end
% % Not resilient to 1,2,7
     
     
% failure = 3; p = length(failure);
% if p > 1
%     error('The malfunctioning reach times only work for 1 lost actuator. Need to add vertices of CW.')
% end
% B = B_bar; B(:,failure) = [];
% C = B_bar(:,failure);
% U = U_bar; U(failure, :) = [];
% W = U_bar(failure, :);



% %%% Quantitative resilience for the driftless system
% options = optimset('linprog');
% options.Display = 'off';
% 
% r_q_driftless = Inf;
% for d = [C, -C] % try d = -C; too
%     % Nominal reach time
%     y = linprog([zeros(m,1); -1], [], [], [B_bar, -d], zeros(n,1), [U_bar(:,1); 0], [U_bar(:,2); Inf], options);
%     lambda = y(end); u_bar = y(1:m);
%     if norm(B_bar*u_bar - lambda*d) > 1e-4
%         error('They should match.')
%     end
%     T_N = 1/lambda;
% 
%     % Malfunctioning reach time
%     lambda_min = Inf;
%     [u_star, w_star] = deal([]);
%     for w = W
%         y = linprog([zeros(m,1); -1], [], [], [B, C, -d], zeros(n,1), [U(:,1); w; 0], [U(:,2); w; Inf], options);
%         if ~isempty(y)
%             lambda = y(end);
%             if lambda < lambda_min
%                 lambda_min = lambda;
%                 u_star = y(1:m-p);
%                 w_star = w;
%                 if norm(B*u_star + C*w_star - lambda_min*d) > 1e-4
%                     error('They should match.')
%                 end
%             end
%         end
%     end
%     T_M = 1/lambda_min;
%     if T_N/T_M < r_q_driftless
%         r_q_driftless = T_N/T_M;
%         T_N_driftless = T_N;
%         T_M_driftless = T_M;
%     end
% end
% 
% 
% 
% %%% Quantitative resilience for linear system
% time_cst_can = 1.6/3;
% time_cst_el = 1/3;
% time_cst_rud = 1.2/3;
% time_cst_flap = 2/3;
% 
% Dubar = diag([1/time_cst_can, 1/time_cst_can, 1/time_cst_el, 1/time_cst_el, 1/time_cst_el, 1/time_cst_el, 1/time_cst_rud, 1/time_cst_flap]);
% Du = Dubar; Du(:, failure) = []; Du(failure, :) = [];
% Dw = diag(Dubar(failure,failure));
% 
% x_goal = zeros(n,1);
% x_0_plus = C;
% x_0_minus = -C;
% 
% % Nominal reach time
% A_bar_tot = [A, B_bar; zeros(m, n), -Dubar];
% B_bar_tot = [zeros(n, m); Dubar];
% 
% [T_N_plus, X_Eaton_plus, ~] = time_optimal_Eaton(A_bar_tot, B_bar_tot, U_bar, [x_0_plus; zeros(m,1)], [x_goal; zeros(m,1)]);
% [T_N_minus, X_Eaton_minus, ~] = time_optimal_Eaton(A_bar_tot, B_bar_tot, U_bar, [x_0_minus; zeros(m,1)], [x_goal; zeros(m,1)]);
% 
% 
% % Malfunctioning reach time
% A_tot = [A,B,C; zeros(m-p, n), -Du, zeros(m-p, p); zeros(p, n+m-p), -Dw];
% B_tot = [zeros(n, m-p); Du; zeros(p, m-p)];
% C_tot = [zeros(n+m-p, p); Dw];
% 
% [T_M_minus, X_Sakawa_minus, ~, ~] = time_optimal_Sakawa(A_tot, B_tot, C_tot, U, W, [x_0_minus; zeros(m,1)], [x_goal; zeros(m,1)]);
% [T_M_plus, X_Sakawa_plus, ~, ~] = time_optimal_Sakawa(A_tot, B_tot, C_tot, U, W, [x_0_plus; zeros(m,1)], [x_goal; zeros(m,1)]);
% 
% r_q_plus = T_N_plus/T_M_plus;
% r_q_minus = T_N_minus/T_M_minus;

