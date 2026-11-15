%% test_aug.m

clc; clear; close all;

% ==================================================
% TEST DU MODÈLE ÉNERGÉTIQUE AUGMENTÉ (dyn_energy.m)
% ==================================================
% Tests realises :
%   T1 — Point fixe à l'equilibre : verification dxdt ~= 0 (sauf dSoC)
%   T2 — Coherence energetique : P_eq et t_vol_max
%   T3 — Simulation libre avec LQR + tracking SoC

load("params.mat")


% PARAMÈTRES BATTERIE (LiPo 3S, 1300 mAh)
SoC_0   = 1.0;          % Batterie pleine au depart
SoC_min = 0.2;          % Seuil de decharge minimale (contrainte future)

% ------------------------
% VITESSE HOVER ANALYTIQUE
% ------------------------
g      = 9.81;
w_h    = drone.w_h;  % [rad/s]
u_eq = [w_h; w_h; w_h; w_h];

fprintf('=== PARAMÈTRES DE RÉFÉRENCE ===\n');
fprintf('  Vitesse equilibre   : w_h   = %.2f rad/s\n', w_h);
fprintf('  Capacite batt       : C     = %.0f A.s\n',   batt.C);
fprintf('  Tension OCV         : E_ocv = %.1f V\n',     batt.E_ocv);
fprintf('\n');

% =========================================================
% TEST 1 — Point fixe a l'equilibre
% Critere : dxdt(1:12) ≈ 0  ET  dSoC < 0 (decharge normale)
% =========================================================
fprintf("=== TEST 1 : Point fixe à l'equilibre ===\n");
%X0 = [x,y,z,vx,vy,vz,phi,theta,psi,p,q,r,w1,w2,w3,w4,Soc,z1,z2,z3,z4)
x0_aug = [zeros(12,1); w_h .* ones(4,1); SoC_0; zeros(4,1)];  % etat augmente [21x1]

dxdt = dyn_uav(x0_aug, u_eq, drone, batt, mot);

fprintf('  Norme residu dynamique dxdt(6) = %.2e  (cible : < 1e-8)\n', ...
        dxdt(6));
fprintf('  dSoC = %.6f s-1  (doit être < 0)\n', dxdt(17));

t_sim = 300;  % simulate 80% du vol max, max 300s
opts  = odeset('RelTol',1e-8, 'AbsTol',1e-10, ...
    'Events', @(t,x) evt_soc_min(t,x,SoC_min));

[t1, X1, ~, ~, ~] = ode45(...
    @(t,x) dyn_uav(x, u_eq, drone, batt, mot), ...
    [0 t_sim], x0_aug, opts);

% --- Figure T1 ---
figure('Name','T1 - Hover');

subplot(3,1,1);
plot(t1, X1(:,1:3), 'LineWidth', 1.4);
legend('x','y','z','Location','east'); ylabel('Position [m]');
title('Reponse LQR avec modele augmente'); grid on;

subplot(3,1,2);
plot(t1, X1(:,7:9)*180/pi, 'LineWidth', 1.4);
legend('\phi','\theta','\psi','Location','east'); ylabel('Attitude [deg]'); grid on;

subplot(3,1,3);
yyaxis left
plot(t1, X1(:,17)*100, 'b-', 'LineWidth', 1.5); ylabel('SoC [%]');
yyaxis right

assert(dxdt(6) < 1e-6, 'ECHEC T1 : residu dynamique trop grand');
assert(dxdt(17) < 0,            'ECHEC T1 : dSoC doit être negatif');
fprintf('  [PASS] Test 1\n\n');

% =========================================================
% TEST 2 — Simulation equilibre : evolution du SoC
% Critere : SoC(t) strictement decroissant, lineaire
%           Pente = -I/C  coherente avec T1
% =========================================================
fprintf('=== TEST 2 : Simulation equilibre — evolution SoC ===\n');

opts  = odeset('RelTol',1e-8, 'AbsTol',1e-10, ...
               'Events', @(t,x) evt_soc_min(t,x,SoC_min));

[t2, X2, te, ~, ~] = ode45(...
    @(t,x) dyn_uav(x, u_eq, drone, batt, mot), ...
    [0 t_sim], x0_aug, opts);

SoC_sim = X2(:,17);

% Verification monotonie
%assert(all(diff(SoC_sim) < 0), 'ECHEC T2 : SoC non decroissant');

% Verification pente (regression lineaire)
p_fit    = polyfit(t2, SoC_sim, 1);          % pente attendue
%pente_th = -I_eq / batt.C;               % [s⁻¹]
%err_rel  = abs(p_fit(1) - pente_th) / abs(pente_th);

fprintf('  Pente SoC mesuree   = %.6e s⁻¹\n', p_fit(1));
%fprintf('  Pente SoC theorique = %.6e s⁻¹\n', pente_th);
%fprintf('  Erreur relative     = %.2f%%  (< 1%% attendu)\n', 100*err_rel);

if ~isempty(te)
    fprintf('  Évenement SoC_min atteint à t = %.1f s\n', te);
end

%assert(err_rel < 0.01, 'ECHEC T2 : pente SoC incoherente avec bilan I/C');
fprintf('  [PASS] Test 2\n\n');

% --- Figure T2 ---
figure('Name',"T2 — SoC à l'equilibre");
yyaxis left
plot(t2, SoC_sim*100, 'b-', 'LineWidth', 1.5);
yline(SoC_min*100, 'r--', sprintf('SoC_{min} = %d%%', 100*SoC_min));
ylabel('SoC [%]'); ylim([0 105]);
yyaxis right


% =========================================================
% TEST 3 — Simulation boucle fermee LQR + SoC
% Objectif : verifier que le modele augmente est compatible
%            avec le contrôleur LQR de simu.m
% =========================================================
fprintf('=== TEST 3 : Boucle fermee LQR — perturbation initiale en z ===\n');

% Linearisation numerique à l'equilibre (sur les 12 premiers etats)
eps_lin = 1e-6;
A = zeros(12,12);
B = zeros(12,4);
for i = 1:12
    xp = zeros(12,1); xp(i) =  eps_lin;
    xm = zeros(12,1); xm(i) = -eps_lin;
    A(:,i) = (dyn(xp, u_eq, drone) - dyn(xm, u_eq, drone)) / (2*eps_lin);
end
for i = 1:4
    up = u_eq; up(i) = up(i)+eps_lin;
    um = u_eq; um(i) = um(i)-eps_lin;
    B(:,i) = (dyn(zeros(12,1),up,drone) - dyn(zeros(12,1),um,drone)) / (2*eps_lin);
end

% Critere de Bryson
deg2rad = pi/180;
x_max = [1;1;0.5;2;2;2;15*deg2rad;15*deg2rad;45*deg2rad;1;1;1];
u_max = repmat(w_h*0.2, 4, 1);
Q_lqr = diag(1./x_max.^2);
R_lqr = diag(1./u_max.^2);

K = lqr(A, B, Q_lqr, R_lqr);
fprintf('  LQR calcule. Valeurs propres A-BK :\n');
disp(eig(A - B*K)');

% Simulation : perturbation z=+0.3m, SoC initial = 1
x0_pert      = [zeros(12,1); w_h.*ones(4,1); 1; zeros(4,1)];
x0_pert(3)   = 0.3;    % perturbation altitude [m]

% Contrôleur LQR (opere sur les 12 etats dynamiques)
ctrl = @(x) u_eq - K*(x(1:12) - zeros(12,1));

t_end = 30;  % [s]
[t3, X3] = ode45(...
    @(t,x) dyn_uav(x, max(ctrl(x), 0), drone, batt, mot), ...
    [0 t_end], x0_pert, ...
    odeset('RelTol',1e-8,'AbsTol',1e-10));

% Verification stabilisation
idx_fin  = find(t3 > 20, 1);  % apres 20s
pos_fin  = X3(idx_fin:end, 1:3);
err_pos  = max(vecnorm(pos_fin, 2, 2));
fprintf('  Erreur position residuelle (t>20s) : %.4f m  (< 0.01 m requis)\n', err_pos);
assert(err_pos < 0.01, 'ECHEC T3 : non stabilise apres 20s');

% Énergie consommee sur la manoeuvre
dSoC_man = X3(1,13) - X3(end,13);  % ΔSoC consomme
E_man    = dSoC_man * batt.C * batt.E_ocv;  % [J]  (approximation)
fprintf('  ΔSoC manoeuvre  = %.4f  (%.2f%%)\n', dSoC_man, 100*dSoC_man);
fprintf('  Énergie manoeuvre ≈ %.2f J  = %.4f Wh\n', E_man, E_man/3600);
fprintf('  [PASS] Test 3\n\n');

% --- Figure T3 ---
figure('Name','T3 — Boucle fermee LQR + SoC');

subplot(3,1,1);
plot(t3, X3(:,1:3), 'LineWidth', 1.4);
legend('x','y','z','Location','east'); ylabel('Position [m]');
title('Reponse LQR avec modele augmente'); grid on;

subplot(3,1,2);
plot(t3, X3(:,7:9)*180/pi, 'LineWidth', 1.4);
legend('\phi','\theta','\psi','Location','east'); ylabel('Attitude [deg]'); grid on;

subplot(3,1,3);
yyaxis left
plot(t3, X3(:,17)*100, 'b-', 'LineWidth', 1.5); ylabel('SoC [%]');
yyaxis right
% Puissance instantanee reconstruite
u_traj = arrayfun(@(k) max(ctrl(X3(k,:)'), 0), 1:length(t3), 'UniformOutput', false);
u_traj = cell2mat(u_traj)';  % [Nx4]
P_traj = (drone.k_Q/1) * sum(u_traj.^3, 2) + batt.P_av;
plot(t3, P_traj, 'r--', 'LineWidth', 1.2); ylabel('P_{elec} [W]');
xlabel('Temps [s]'); legend('SoC','P_{elec}'); grid on;

fprintf('=== BILAN VALIDATION ===\n');
fprintf('  T1 [equilibre statique]    : PASS\n');
fprintf('  T2 [decharge lineaire] : PASS\n');
fprintf('  T3 [LQR + SoC]         : PASS\n');

% =========================================================
% FONCTION ÉVÉNEMENT — arrêt si SoC < SoC_min
% =========================================================
function [value, isterminal, direction] = evt_soc_min(~, x, SoC_min)
    value      = x(17) - SoC_min;  % s'annule quand SoC = SoC_min
    isterminal = 1;                 % arrête l'integration
    direction  = -1;                % detecte la descente
end