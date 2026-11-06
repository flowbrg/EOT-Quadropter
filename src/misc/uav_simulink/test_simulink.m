%% test_aug.m  --  Validation du modele Simulink sim_aug.slx (T1, T2)
clc; clear; close all;

load('params.mat');                 % drone, batt, mot
mdl     = 'sim_aug';
xi0     = [0; 0; 0];                % [m]  position initiale
SoC_0   = 1.0;                      % [-]
SoC_min = 0.2;                      % [-]

% Tolerances (heuristiques, cf. tableau des criteres)
tol_w     = 1e-3;                   % [-]      ecart relatif omega_i / w_h
tol_acc   = 1e-3;                   % [m/s^2]
tol_att   = 1e-4;                   % [rad], [rad/s]
tol_lin   = 0.02;                   % [-]      ecart a la droite de SoC
tol_slope = 0.15;                   % [-]      PROVISOIRE

% --- Equilibre analytique ---
g    = 9.81;
w_h  = sqrt(drone.m*g/(4*drone.k_T));   % [rad/s]
u_eq = w_h*ones(4,1);                    % [4x1]

fprintf('=== PARAMETRES DE REFERENCE ===\n');
fprintf('  w_h = %.2f rad/s | C = %.0f A.s | E_ocv = %.1f V\n\n', ...
    w_h, batt.C, batt.E_ocv);

% --- Bilan de puissance analytique (inchange) ---
P_mec_eq  = drone.k_Q*sum(u_eq.^3);                 % [W]
P_elec_eq = P_mec_eq/mot.eta_m + batt.P_av;      % [W]
I_eq      = P_elec_eq/batt.E_ocv;                % [A]
t_vol_max = batt.E_ocv*batt.C*(1-SoC_min)/P_elec_eq;   % [s]

%% ===== TEST 1 : regime permanent a w_ref = w_h =====
fprintf('=== TEST 1 : regime permanent au hover ===\n');
t_f1 = 10;  t_s1 = 5;               % [s] fenetre d'analyse [t_s1, t_f1]
[t1, Y1] = run_sim_eq(mdl, u_eq, t_f1, xi0, u_eq, SoC_0);

dt = 1e-2;
tu = (t_s1:dt:t_f1).';
Yu = interp1(t1, Y1, tu, 'linear');          % [Mx17], pas uniforme
acc = diff(Yu(:,4:6))/dt;                    % [(M-1)x3] m/s^2

r.w   = max(abs(Yu(:,13:16) - w_h), [], 'all')/w_h;
r.acc = max(vecnorm(acc,2,2));
r.att = max(abs(Yu(:,[7 8 10 11 12])), [], 'all');
r.vend= norm(Yu(end,4:6));
pS    = polyfit(tu, Yu(:,17), 1);
r.trans = max(abs(Y1(:,13:16) - w_h), [], 'all')/w_h;   % tout le run

fprintf('  max|w_i-w_h|/w_h (fenetre) = %.2e  (< %.0e)\n', r.w,   tol_w);
fprintf('  max||dv/dt||               = %.2e m/s^2 (< %.0e)\n', r.acc, tol_acc);
fprintf('  max|phi,theta,p,q,r|       = %.2e  (< %.0e)\n', r.att, tol_att);
fprintf('  dSoC/dt (fenetre)          = %.3e s^-1 (< 0)\n', pS(1));
fprintf('  [diag] ||v(t_f)||          = %.2e m/s\n', r.vend);
fprintf('  [diag] transitoire max w   = %.2e (relatif, sur tout le run)\n', r.trans);

assert(r.w   < tol_w,   'ECHEC T1-a : w_i non stabilise a w_h');
assert(r.acc < tol_acc, 'ECHEC T1-b : acceleration non nulle');
assert(r.att < tol_att, 'ECHEC T1-c : attitude/vitesses angulaires non nulles');
assert(pS(1) < 0,       'ECHEC T1-d : SoC ne decroit pas');
if r.vend > 1e-3
    warning(['||v(t_f)|| > 1e-3 m/s : conditions initiales internes ' ...
        'probablement incoherentes avec le hover (hypothese H3).']);
end
fprintf('  [PASS] Test 1\n\n');

%% ===== TEST 2 : decharge au hover =====
fprintf('=== TEST 2 : evolution du SoC au hover ===\n');
t_sim2 = min(300, 0.8*t_vol_max);   % [s]
t_s2   = 5;                         % [s] exclusion du transitoire
[t2, Y2] = run_sim_eq(mdl, u_eq, t_sim2, xi0, u_eq, SoC_0);
SoC = Y2(:,17);

assert(all(diff(SoC) < 0),   'ECHEC T2-a : SoC non strictement decroissant');
assert(SoC(end) > SoC_min,   'ECHEC T2 : SoC_min atteint pendant le test');

m   = t2 > t_s2;
p   = polyfit(t2(m), SoC(m), 1);                      % p(1) : pente [s^-1]
dev = max(abs(SoC(m) - polyval(p, t2(m)))) / abs(SoC(find(m,1,'last')) - SoC(find(m,1)));
pente_th = -I_eq/batt.C;                      % [s^-1]
err_rel  = abs(p(1) - pente_th)/abs(pente_th);

fprintf('  Pente mesuree   = %.6e s^-1\n', p(1));
fprintf('  Pente analytique= %.6e s^-1  (I_eq = %.2f A)\n', pente_th, I_eq);
fprintf('  Ecart relatif   = %.2f %%  (< %.0f %%, provisoire)\n', 100*err_rel, 100*tol_slope);
fprintf('  Ecart a la droite = %.2f %%  (< %.0f %%)\n', 100*dev, 100*tol_lin);

assert(dev      < tol_lin,   'ECHEC T2-b : decharge non lineaire');
assert(err_rel  < tol_slope, 'ECHEC T2-c : pente incoherente avec le bilan analytique');
fprintf('  [PASS] Test 2\n\n');

figure('Name','T2 - SoC au hover');
plot(t2, 100*SoC, 'b-', t2(m), 100*polyval(p,t2(m)), 'k--', 'LineWidth', 1.3);
yline(100*SoC_min, 'r--', sprintf('SoC_{min} = %d%%', 100*SoC_min));
xlabel('t [s]'); ylabel('SoC [%]'); grid on;
legend('Simulink','Regression (fenetre)','Location','southwest');
title('Decharge au hover (modele Simulink)');