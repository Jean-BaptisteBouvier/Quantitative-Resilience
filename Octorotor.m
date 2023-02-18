%%%%%%%%%%%%%%%%%%%%%%%%%% 8 rotors non-coplanar: 4 horizontal + 4 vertical
% Based on "A New UAV Configuration Having Eight rotors"
% and "Modelling and Real-Time Control Stabilization"

%%% Numerical value from the star-shaped octorotor described in
% "Active Fault-tolerant Control of Octorotor UAV using Dynamic Control Allocation"
% and "Integral LQR control of a star-shaped octorotor"




function [B_t, B_r] = Octorotor(psi) % [rad]

theta = 0; % roll angle
phi = 0; % pitch angle are zero because the UAV stays flat

% Inertia
Ix = 44e-3; % [kg m^2]
Iy = 44e-3; % [kg m^2]
Iz = 2 * 44e-3; % [kg m^2]
I_rotor = 90e-6; % [kg m^2]

c = 1e-5; % [N s^2] thrust coefficient
d = 0.3e-6; % [m s^2] drag coefficient
l = 0.4; % [m] arm length
m = 1.64; % [kg] mass
g = 9.81; % [m/s^2] gravity
b = 0.64; % estimated slope

%%%%%%%%%%%%%%%%%%%%% Translational model (angles fixed)

% Rotation matrix
R = [cos(psi)*cos(theta), cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi), cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi);
     sin(psi)*cos(theta), sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi);
     -sin(theta),         cos(theta)*sin(phi),                              cos(theta)*cos(phi)];
 

B_t = R/m * [0, 0, 0, 0, 1, -1, 0,  0;
             0, 0, 0, 0, 0,  0, 1, -1;
             1, 1, 1, 1, b,  b, b,  b];

% ddot [x; y; z] = B_t * [f_1'; f_2'; f_3'; f_4'; f_5; f_6; f_7; f_8]
% with  -mg/4 <= f_i' <= k w_max^2 - mg/4  for i = 1,2,3,4
% and  0 <= f_j <= k w_max^2   for j = 5,6,7,8



%%%%%%%%%%%%%%%%%%%%% Rotational model

B_r = [-l*c/Ix, 0, l*c/Ix, 0, 0, 0, l*b*c/Ix, -l*b*c/Ix;
	   0, l*c/Iy, 0, -l*c/Iy, l*b*c/Iy, -l*b*c/Iy, 0, 0;
	   -d/Iz, d/Iz, -d/Iz, d/Iz, 0, 0, 0, 0];

% ddot [phi, theta, psi] = B_r * [w_1^2, w_2^2, w_3^2, w_4^2, w_5^2, w_6^2, w_7^2, w_8^2]

end
