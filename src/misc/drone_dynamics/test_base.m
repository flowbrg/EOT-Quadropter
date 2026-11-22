%% test_base.m

clc; clear; close all;

% =====================================
% DYNAMIC MODEL TEST (dyn.m)
% =====================================
% Tests performed:
%   T1 — Equilibrium fixed point: check dxdt ~= 0
%   T2 — Free simulation with LQR

load("params.mat")
params = drone;

% ------------------------
% ANALYTICAL HOVER SPEED
% ------------------------
g      = 9.81;
w_h = sqrt((drone.m*g) / (4*drone.k_T));
u_eq = [w_h; w_h; w_h; w_h];

fprintf('=== REFERENCE PARAMETERS ===\n');
fprintf('  Equilibrium speed   : w_h   = %.2f rad/s\n', w_h);
fprintf('\n');

% =====================================
% TEST 1 — Equilibrium fixed point
% Criterion: dxdt(1:12) ~= 0
% =====================================
fprintf("=== TEST 1 : Equilibrium fixed point weight/thrust===\n");

x0 = zeros(12,1);
dxdt = dyn(x0, u_eq, params);
fprintf('  Dynamic residual norm ||dxdt(1:12)|| = %.2e  (target : < 1e-8)\n', ...
        norm(dxdt(1:12)));
assert(norm(dxdt(1:12)) < 1e-8, 'FAIL T1 : dynamic residual too large');

% --- Free simulation ---
[t, X] = ode45(@(t,x) dyn(x, u_eq, params), [0 5], x0);

figure('Name',"T1 - theoretical equilibrium position");
plot(t, X(:,3)); xlabel('t [s]'); ylabel('z [m]');
title("Altitude at equilibrium");

% =====================================
% TEST 2 — Closed-loop LQR simulation
% =====================================

% Numerical linearization at equilibrium
eps = 1e-6;
A = zeros(12,12);
B = zeros(12,4);
for i = 1:12
    xp = x0; xp(i) = xp(i)+eps;
    xm = x0; xm(i) = xm(i)-eps;
    A(:,i) = (dyn(xp,u_eq,params) - dyn(xm,u_eq,params)) / (2*eps);
end
for i = 1:4
    up = u_eq; up(i) = up(i)+eps;
    um = u_eq; um(i) = um(i)-eps;
    B(:,i) = (dyn(x0,up,params) - dyn(x0,um,params)) / (2*eps);
end

% Bryson's rule
deg2rad = pi/180;
x_max = [1; 1; 0.5; 2; 2; 2; 15*deg2rad; 15*deg2rad; 45*deg2rad; 1; 1; 1];
u_max = repmat(w_h*0.2, 4, 1); % 20% of w_eq as max variation

Q = diag(1./x_max.^2);
R = diag(1./u_max.^2);

% LQR
K = lqr(A, B, Q, R);
fprintf('  LQR computed. Eigenvalues of A-BK :\n');
disp(eig(A - B*K)');

% Closed-loop simulation
x0_pert = zeros(12,1);
x0_pert(3) = 0.3;       % initial perturbation in z

% LQR controller (operates on the 12 dynamic states)
ctrl = @(x) u_eq - K*(x - x0);

t_end = 30; % [s]
[t, X] = ode45(@(t,x) dyn(x, ctrl(x), params), [0 t_end], x0_pert);

% Stabilization check
idx_fin  = find(t > 20, 1);  % after 20s
pos_fin  = X(idx_fin:end, 1:3);
err_pos  = max(vecnorm(pos_fin, 2, 2));
fprintf('  Residual position error (t>20s) : %.4f m  (< 0.01 m required)\n', err_pos);
assert(err_pos < 0.01, 'FAIL T3 : not stabilized after 20s');

figure('Name','T2 — Closed-loop LQR');
subplot(3,1,1); plot(t, X(:,1:3)); legend('x','y','z'); ylabel('Position [m]');
subplot(3,1,2); plot(t, X(:,4:6)); legend('vx','vy','vz'); ylabel('Velocity [m/s]');
subplot(3,1,3); plot(t, X(:,7:9)*180/pi); legend('\phi','\theta','\psi'); ylabel('Attitude [deg]');
xlabel('t [s]');

fprintf('=== VALIDATION SUMMARY ===\n');
fprintf('  T1 [static equilibrium]    : PASS\n');
fprintf('  T2 [LQR] : PASS\n');