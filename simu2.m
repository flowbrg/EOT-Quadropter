clc; clear; close all;

% ==================================================
% TEST DU MODÈLE ÉNERGÉTIQUE AUGMENTÉ (dyn_energy.m)
% ==================================================
% Tests réalisés :
%   T1 — Point fixe au hover : vérification dxdt ~= 0 (sauf dSoC)
%   T2 — Cohérence énergétique : P_hover et t_vol_max
%   T3 — Simulation libre avec LQR + tracking SoC

init_params

% PARAMÈTRES BATTERIE (LiPo 3S, 1300 mAh)
SoC_0   = 1.0;          % Batterie pleine au départ
SoC_min = 0.2;          % Seuil de décharge minimale (contrainte future)

% ------------------------
% VITESSE HOVER ANALYTIQUE
% ------------------------
g      = 9.81;
w_h    = sqrt((params(1)*g) / (4*params(2)));  % [rad/s]
u_hover = [w_h; w_h; w_h; w_h];

fprintf('=== PARAMÈTRES DE RÉFÉRENCE ===\n');
fprintf('  Vitesse hover   : w_h   = %.2f rad/s\n', w_h);
fprintf('  Capacité batt   : C     = %.0f A.s\n',   C_batt);
fprintf('  Tension OCV     : E_ocv = %.1f V\n',     E_ocv);
fprintf('\n');

% =========================================================
% TEST 1 — Point fixe au hover
% Critère : dxdt(1:12) ≈ 0  ET  dSoC < 0 (décharge normale)
% =========================================================
fprintf('=== TEST 1 : Point fixe au hover ===\n');

x0_aug = [zeros(12,1); SoC_0];  % état augmenté [13x1]

dxdt = dyn_energy(x0_aug, u_hover, params, batt_params);

fprintf('  Norme résidu dynamique ||dxdt(1:12)|| = %.2e  (cible : < 1e-8)\n', ...
        norm(dxdt(1:12)));
fprintf('  dSoC = %.6f s-1  (doit être < 0)\n', dxdt(13));

% Puissance au hover (recalcul explicite pour vérification)
P_mec_hover  = params(3) * sum(u_hover.^3);        % k_Q * 4*w_h^3  [W]
P_elec_hover = P_mec_hover / eta_m + P_av;          % [W]
I_hover      = P_elec_hover / E_ocv;                % [A]

fprintf('\n  -- Bilan énergétique hover --\n');
fprintf('  P_mec  = %.2f W\n', P_mec_hover);
fprintf('  P_elec = %.2f W  (avec eta_m=%.2f, P_av=%.1f W)\n', ...
        P_elec_hover, eta_m, P_av);
fprintf('  I      = %.2f A\n', I_hover);
fprintf('  Chute ohmique R0*I/E_ocv = %.1f%%  (< 5%% requis)\n', ...
        100*R0*I_hover/E_ocv);

% Temps de vol maximum théorique
t_vol_max = (E_ocv * C_batt * (1 - SoC_min)) / P_elec_hover;  % [s]
fprintf('  t_vol_max (SoC: %.0f%%→%.0f%%) = %.0f s = %.1f min\n', ...
        100*SoC_0, 100*SoC_min, t_vol_max, t_vol_max/60);

assert(norm(dxdt(1:12)) < 1e-8, 'ECHEC T1 : résidu dynamique trop grand');
assert(dxdt(13) < 0,            'ECHEC T1 : dSoC doit être négatif');
fprintf('  [PASS] Test 1\n\n');

% =========================================================
% TEST 2 — Simulation hover pur : évolution du SoC
% Critère : SoC(t) strictement décroissant, linéaire
%           Pente = -I/C_batt  cohérente avec T1
% =========================================================
fprintf('=== TEST 2 : Simulation hover — évolution SoC ===\n');

t_sim = min(300, 0.8*t_vol_max);  % simulate 80% du vol max, max 300s
opts  = odeset('RelTol',1e-8, 'AbsTol',1e-10, ...
               'Events', @(t,x) evt_soc_min(t,x,SoC_min));

[t2, X2, te, ~, ~] = ode45(...
    @(t,x) dyn_energy(x, u_hover, params, batt_params), ...
    [0 t_sim], x0_aug, opts);

SoC_sim = X2(:,13);

% Vérification monotonie
assert(all(diff(SoC_sim) < 0), 'ECHEC T2 : SoC non décroissant');

% Vérification pente (régression linéaire)
p_fit    = polyfit(t2, SoC_sim, 1);          % pente attendue
pente_th = -I_hover / C_batt;               % [s⁻¹]
err_rel  = abs(p_fit(1) - pente_th) / abs(pente_th);

fprintf('  Pente SoC mesurée   = %.6e s⁻¹\n', p_fit(1));
fprintf('  Pente SoC théorique = %.6e s⁻¹\n', pente_th);
fprintf('  Erreur relative     = %.2f%%  (< 1%% attendu)\n', 100*err_rel);

if ~isempty(te)
    fprintf('  Événement SoC_min atteint à t = %.1f s\n', te);
end

assert(err_rel < 0.01, 'ECHEC T2 : pente SoC incohérente avec bilan I/C');
fprintf('  [PASS] Test 2\n\n');

% --- Figure T2 ---
figure('Name','T2 — SoC au hover');
yyaxis left
plot(t2, SoC_sim*100, 'b-', 'LineWidth', 1.5);
yline(SoC_min*100, 'r--', sprintf('SoC_{min} = %d%%', 100*SoC_min));
ylabel('SoC [%]'); ylim([0 105]);
yyaxis right
% Puissance instantanée (constante au hover)
plot(t2, ones(size(t2))*P_elec_hover, 'k:', 'LineWidth', 1.2);
ylabel('P_{elec} [W]');
xlabel('Temps [s]');
title('Évolution du SoC au hover (validation linéarité)');
legend('SoC simulé','Seuil SoC_{min}','P_{elec}','Location','west');
grid on;

% =========================================================
% TEST 3 — Simulation boucle fermée LQR + SoC
% Objectif : vérifier que le modèle augmenté est compatible
%            avec le contrôleur LQR de simu.m
% =========================================================
fprintf('=== TEST 3 : Boucle fermée LQR — perturbation initiale en z ===\n');

% Linéarisation numérique au hover (sur les 12 premiers états)
eps_lin = 1e-6;
A = zeros(12,12);
B = zeros(12,4);
for i = 1:12
    xp = zeros(12,1); xp(i) =  eps_lin;
    xm = zeros(12,1); xm(i) = -eps_lin;
    A(:,i) = (dyn(xp, u_hover, params) - dyn(xm, u_hover, params)) / (2*eps_lin);
end
for i = 1:4
    up = u_hover; up(i) = up(i)+eps_lin;
    um = u_hover; um(i) = um(i)-eps_lin;
    B(:,i) = (dyn(zeros(12,1),up,params) - dyn(zeros(12,1),um,params)) / (2*eps_lin);
end

% Critère de Bryson
deg2rad = pi/180;
x_max = [1;1;0.5;2;2;2;15*deg2rad;15*deg2rad;45*deg2rad;1;1;1];
u_max = repmat(w_h*0.2, 4, 1);
Q_lqr = diag(1./x_max.^2);
R_lqr = diag(1./u_max.^2);

K = lqr(A, B, Q_lqr, R_lqr);
fprintf('  LQR calculé. Valeurs propres A-BK :\n');
disp(eig(A - B*K)');

% Simulation : perturbation z=+0.3m, SoC initial = 1
x0_pert      = zeros(13,1);
x0_pert(3)   = 0.3;    % perturbation altitude [m]
x0_pert(13)  = SoC_0;  % batterie pleine

% Contrôleur LQR (opère sur les 12 états dynamiques)
ctrl = @(x) u_hover - K*(x(1:12) - zeros(12,1));

t_end = 30;  % [s]
[t3, X3] = ode45(...
    @(t,x) dyn_energy(x, max(ctrl(x), 0), params, batt_params), ...
    [0 t_end], x0_pert, ...
    odeset('RelTol',1e-8,'AbsTol',1e-10));

% Vérification stabilisation
idx_fin  = find(t3 > 20, 1);  % après 20s
pos_fin  = X3(idx_fin:end, 1:3);
err_pos  = max(vecnorm(pos_fin, 2, 2));
fprintf('  Erreur position résiduelle (t>20s) : %.4f m  (< 0.01 m requis)\n', err_pos);
assert(err_pos < 0.01, 'ECHEC T3 : non stabilisé après 20s');

% Énergie consommée sur la manoeuvre
dSoC_man = X3(1,13) - X3(end,13);  % ΔSoC consommé
E_man    = dSoC_man * C_batt * E_ocv;  % [J]  (approximation)
fprintf('  ΔSoC manoeuvre  = %.4f  (%.2f%%)\n', dSoC_man, 100*dSoC_man);
fprintf('  Énergie manoeuvre ≈ %.2f J  = %.4f Wh\n', E_man, E_man/3600);
fprintf('  [PASS] Test 3\n\n');

% --- Figure T3 ---
figure('Name','T3 — Boucle fermée LQR + SoC');

subplot(3,1,1);
plot(t3, X3(:,1:3), 'LineWidth', 1.4);
legend('x','y','z','Location','east'); ylabel('Position [m]');
title('Réponse LQR avec modèle augmenté'); grid on;

subplot(3,1,2);
plot(t3, X3(:,7:9)*180/pi, 'LineWidth', 1.4);
legend('\phi','\theta','\psi','Location','east'); ylabel('Attitude [deg]'); grid on;

subplot(3,1,3);
yyaxis left
plot(t3, X3(:,13)*100, 'b-', 'LineWidth', 1.5); ylabel('SoC [%]');
yyaxis right
% Puissance instantanée reconstruite
u_traj = arrayfun(@(k) max(ctrl(X3(k,:)'), 0), 1:length(t3), 'UniformOutput', false);
u_traj = cell2mat(u_traj)';  % [Nx4]
P_traj = (params(3)/eta_m) * sum(u_traj.^3, 2) + P_av;
plot(t3, P_traj, 'r--', 'LineWidth', 1.2); ylabel('P_{elec} [W]');
xlabel('Temps [s]'); legend('SoC','P_{elec}'); grid on;

fprintf('=== BILAN VALIDATION ===\n');
fprintf('  T1 [hover statique]    : PASS\n');
fprintf('  T2 [décharge linéaire] : PASS\n');
fprintf('  T3 [LQR + SoC]         : PASS\n');

% =========================================================
% FONCTION ÉVÉNEMENT — arrêt si SoC < SoC_min
% =========================================================
function [value, isterminal, direction] = evt_soc_min(~, x, SoC_min)
    value      = x(13) - SoC_min;  % s'annule quand SoC = SoC_min
    isterminal = 1;                 % arrête l'intégration
    direction  = -1;                % détecte la descente
end