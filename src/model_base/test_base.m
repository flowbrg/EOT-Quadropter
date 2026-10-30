%% test_base.m

clc; clear; close all;

% ================================
% TEST DU MODÈLE DYNAMIQUE (dyn.m)
% ================================
% Tests realises :
%   T1 — Point fixe à l'equilibre : verification dxdt ~= 0
%   T2 — Simulation libre avec LQR

load("params.mat")
params = drone;

% ------------------------
% VITESSE HOVER ANALYTIQUE
% ------------------------
g      = 9.81;
w_h = sqrt((drone.m*g) / (4*drone.k_T))
u_eq = [w_h; w_h; w_h; w_h];

fprintf('=== PARAMÈTRES DE RÉFÉRENCE ===\n');
fprintf('  Vitesse equilibre   : w_h   = %.2f rad/s\n', w_h);
fprintf('\n');

% =================================
% TEST 1 — Point fixe à l'equilibre
% Critere : dxdt(1:12) ~= 0
% =================================
fprintf("=== TEST 1 : Point fixe à l'equilibre poid/poussee===\n");

x0 = zeros(12,1);
dxdt = dyn(x0, u_eq, params);
fprintf('  Norme residu dynamique ||dxdt(1:12)|| = %.2e  (cible : < 1e-8)\n', ...
        norm(dxdt(1:12)));
assert(norm(dxdt(1:12)) < 1e-8, 'ECHEC T1 : residu dynamique trop grand');

% --- Simulation libre ---
[t, X] = ode45(@(t,x) dyn(x, u_eq, params), [0 5], x0);

figure('Name',"T1 - position à l'equilibre theorique");
plot(t, X(:,3)); xlabel('t [s]'); ylabel('z [m]');
title("Altitude à l'equilibre");

% =====================================
% TEST 2 — Simulation boucle fermee LQR
% =====================================

% Linearisation numerique à l'equilibre
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

% Critere de Bryson
deg2rad = pi/180;
x_max = [1; 1; 0.5; 2; 2; 2; 15*deg2rad; 15*deg2rad; 45*deg2rad; 1; 1; 1];
u_max = repmat(w_h*0.2, 4, 1); % 20% de w_eq comme variation max

Q = diag(1./x_max.^2);
R = diag(1./u_max.^2);

% LQR
K = lqr(A, B, Q, R);
fprintf('  LQR calcule. Valeurs propres A-BK :\n');
disp(eig(A - B*K)');

% Simulation en boucle fermee
x0_pert = zeros(12,1);
x0_pert(3) = 0.3;       % perturbation initiale en z

% Contrôleur LQR (opere sur les 12 etats dynamiques)
ctrl = @(x) u_eq - K*(x - x0);

t_end = 30; % [s]
[t, X] = ode45(@(t,x) dyn(x, ctrl(x), params), [0 t_end], x0_pert);

% Verification stabilisation
idx_fin  = find(t > 20, 1);  % apres 20s
pos_fin  = X(idx_fin:end, 1:3);
err_pos  = max(vecnorm(pos_fin, 2, 2));
fprintf('  Erreur position residuelle (t>20s) : %.4f m  (< 0.01 m requis)\n', err_pos);
assert(err_pos < 0.01, 'ECHEC T3 : non stabilise apres 20s');

figure('Name','T2 — Boucle fermee LQR');
subplot(3,1,1); plot(t, X(:,1:3)); legend('x','y','z'); ylabel('Position [m]');
subplot(3,1,2); plot(t, X(:,4:6)); legend('vx','vy','vz'); ylabel('Vitesse [m/s]');
subplot(3,1,3); plot(t, X(:,7:9)*180/pi); legend('\phi','\theta','\psi'); ylabel('Attitude [deg]');
xlabel('t [s]');

fprintf('=== BILAN VALIDATION ===\n');
fprintf('  T1 [equilibre statique]    : PASS\n');
fprintf('  T2 [LQR] : PASS\n');