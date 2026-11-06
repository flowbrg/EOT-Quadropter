%% test_t3.m -- T3 : LQR en boucle fermee sur le modele Simulink complet
clc; clear; close all;

load('params.mat');                 % drone, batt, mot
load('lqr_hover.mat');              % K [4x12], w_h
mdl = 'sim_aug';  harness = 'sim_aug_lqr';
u_eq = w_h*ones(4,1);               % [4x1] rad/s (lu par le harnais)
xi0  = [0; 0; 0.3];                 % [m] perturbation initiale (H9)
SoC_0 = 1;  w0_motor = u_eq;        % lus par le modele (workspace de base)

build_harness(mdl, harness);

t_end = 30;                         % [s]
in = Simulink.SimulationInput(harness);
in = in.setModelParameter('Solver','ode45','RelTol','1e-8','AbsTol','1e-10', ...
    'StopTime',num2str(t_end), 'SaveTime','on','SaveOutput','on', ...
    'OutputSaveName','yout','SaveFormat','Dataset');
in = in.setVariable('K', K);
in = in.setVariable('u_eq', u_eq);
in = in.setVariable('xi0', xi0);
in = in.setVariable('w0_motor', w0_motor);
in = in.setVariable('SoC_0', SoC_0);

out = sim(in);
if ~isempty(out.ErrorMessage), error('T3:sim', 'Erreur Simulink : %s', out.ErrorMessage); end

[t, Y] = extract_outputs(out.yout, [3 3 3 3 4 1 4]);   % Y [Nx21] : 17 etats + u_cmd(4)
xi = Y(:,1:3);  eta = Y(:,7:9);  SoC = Y(:,17);  u = Y(:,18:21);

%% Criteres
mask   = t > 20;
err_xi = max(vecnorm(xi(mask,:),2,2));              % [m]
ang_max = max(abs(eta(:,1:2)),[],'all')*180/pi;     % [deg]
frac_sat = mean(any(u <= 0, 2));                    % fraction du temps en saturation basse
dSoC = SoC(1) - SoC(end);
E_J  = dSoC*batt.C*batt.E_ocv;           % [J] approximation d'origine

fprintf('=== TEST 3 : LQR + modele Simulink complet ===\n');
fprintf('  T3-a max||xi|| (t>20 s) = %.4f m   (< 0.01 requis)\n', err_xi);
fprintf('  T3-b max|phi,theta|     = %.2f deg (< 15 requis)\n', ang_max);
fprintf('  T3-c saturation basse   = %.2f %% du temps (info)\n', 100*frac_sat);
fprintf('  T3-d dSoC = %.4f (%.2f %%), E ~ %.2f J = %.4f Wh (info)\n', ...
    dSoC, 100*dSoC, E_J, E_J/3600);

assert(err_xi   < 0.01, 'ECHEC T3-a : non stabilise apres 20 s');
assert(ang_max  < 15,   'ECHEC T3-b : angle > 15 deg');
fprintf('  [PASS] Test 3\n');

%% Figures
figure('Name','T3 - LQR + modele complet');
subplot(4,1,1); plot(t, xi, 'LineWidth',1.3); grid on; ylabel('\xi [m]'); legend('x','y','z');
subplot(4,1,2); plot(t, eta*180/pi, 'LineWidth',1.3); grid on; ylabel('\eta [deg]'); legend('\phi','\theta','\psi');
subplot(4,1,3); plot(t, u, 'LineWidth',1.3); grid on; ylabel('\omega_{ref} [rad/s]');
subplot(4,1,4); plot(t, 100*SoC, 'b','LineWidth',1.3); grid on; ylabel('SoC [%]'); xlabel('t [s]');

%% Fonction locale (Dataset -> matrice) : meme logique que run_sim_eq
function [t, Y] = extract_outputs(ys, nCol)
t = ys{1}.Values.Time;
Y = zeros(numel(t), sum(nCol));  c = 0;
for k = 1:numel(nCol)
    d = ys{k}.Values.Data;
    if ndims(d) == 3 && size(d,1) == nCol(k) && size(d,3) == numel(t)
        d = permute(d, [3 1 2]);
    end
    d = reshape(d, numel(t), []);
    assert(size(d,2) == nCol(k), 'Outport %d : %d colonnes au lieu de %d.', k, size(d,2), nCol(k));
    Y(:, c+1:c+nCol(k)) = d;  c = c + nCol(k);
end
end