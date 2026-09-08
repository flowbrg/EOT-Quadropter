clc; clear; close all;

% ================================
% TEST DU MODÈLE DYNAMIQUE (dyn.m)
% ================================
% Tests réalisés :
%   T1 — Point fixe au hover : vérification dxdt ~= 0
%   T2 — Simulation libre avec LQR

init_params

% ------------------------
% VITESSE HOVER ANALYTIQUE
% ------------------------
g      = 9.81;
w_h = sqrt((params(1)*g) / (4*params(2)));
u_hover = [w_h; w_h; w_h; w_h];

fprintf('=== PARAMÈTRES DE RÉFÉRENCE ===\n');
fprintf('  Vitesse hover   : w_h   = %.2f rad/s\n', w_h);
fprintf('\n');

% ============================
% TEST 1 — Point fixe au hover
% Critère : dxdt(1:12) ~= 0
% ============================
fprintf("=== TEST 1 : Point fixe à l'équilibre poid/poussée===\n");

x0 = zeros(12,1);
dxdt = dyn(x0, u_hover, params);
fprintf('  Norme résidu dynamique ||dxdt(1:12)|| = %.2e  (cible : < 1e-8)\n', ...
        norm(dxdt(1:12)));
assert(norm(dxdt(1:12)) < 1e-8, 'ECHEC T1 : résidu dynamique trop grand');

% --- Simulation libre ---
[t, X] = ode45(@(t,x) dyn(x, u_hover, params), [0 5], x0);

figure('Name',"T1 - position à l'équilibre théorique");
plot(t, X(:,3)); xlabel('t [s]'); ylabel('z [m]');
title("Altitude à l'équilibre");

% =====================================
% TEST 2 — Simulation boucle fermée LQR
% =====================================

% Linéarisation numérique à l'équilibre
eps = 1e-6;
A = zeros(12,12);
B = zeros(12,4);
for i = 1:12
    xp = x0; xp(i) = xp(i)+eps;
    xm = x0; xm(i) = xm(i)-eps;
    A(:,i) = (dyn(xp,u_hover,params) - dyn(xm,u_hover,params)) / (2*eps);
end
for i = 1:4
    up = u_hover; up(i) = up(i)+eps;
    um = u_hover; um(i) = um(i)-eps;
    B(:,i) = (dyn(x0,up,params) - dyn(x0,um,params)) / (2*eps);
end

% Critère de Bryson
deg2rad = pi/180;
x_max = [1; 1; 0.5; 2; 2; 2; 15*deg2rad; 15*deg2rad; 45*deg2rad; 1; 1; 1];
u_max = repmat(w_h*0.2, 4, 1); % 20% de w_hover comme variation max

Q = diag(1./x_max.^2);
R = diag(1./u_max.^2);

% LQR
K = lqr(A, B, Q, R);
fprintf('  LQR calculé. Valeurs propres A-BK :\n');
disp(eig(A - B*K)');

% Simulation en boucle fermee
x0_pert = zeros(12,1);
x0_pert(3) = 0.3;       % perturbation initiale en z

% Contrôleur LQR (opère sur les 12 états dynamiques)
ctrl = @(x) u_hover - K*(x - x0);

t_end = 30; % [s]
[t, X] = ode45(@(t,x) dyn(x, ctrl(x), params), [0 t_end], x0_pert);

% Vérification stabilisation
idx_fin  = find(t > 20, 1);  % après 20s
pos_fin  = X(idx_fin:end, 1:3);
err_pos  = max(vecnorm(pos_fin, 2, 2));
fprintf('  Erreur position résiduelle (t>20s) : %.4f m  (< 0.01 m requis)\n', err_pos);
assert(err_pos < 0.01, 'ECHEC T3 : non stabilisé après 20s');

figure('Name','T3 — Boucle fermée LQR + SoC');
subplot(3,1,1); plot(t, X(:,1:3)); legend('x','y','z'); ylabel('Position [m]');
subplot(3,1,2); plot(t, X(:,4:6)); legend('vx','vy','vz'); ylabel('Vitesse [m/s]');
subplot(3,1,3); plot(t, X(:,7:9)*180/pi); legend('\phi','\theta','\psi'); ylabel('Attitude [deg]');
xlabel('t [s]');

fprintf('=== BILAN VALIDATION ===\n');
fprintf('  T1 [hover statique]    : PASS\n');
fprintf('  T2 [LQR] : PASS\n');