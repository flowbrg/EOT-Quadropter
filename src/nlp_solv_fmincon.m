%% Optimisation de trajectoire - Quadrirotor 3D, energie minimale, SANS obstacle
%  Optimization Toolbox, approche problem-based (optimvar + solve -> fmincon)
%
%  Modele d'etat 16 etats : [x y z vx vy vz phi theta psi p q r w1 w2 w3 w4]
%  Commandes              : U = [dw1..dw4]  accelerations angulaires [rad/s^2]
%
%  Equivalent de nlp_solv2.m (CasADi/IPOPT), sans la partie batiments.
%  Prerequis : Optimization Toolbox ; params.mat (drone, mot) ; scenario.mat (scen).
%
%  Derivees :
%   - gradient de l'objectif et Jacobiennes des contraintes : differentiation
%     automatique (AD) de solve, tant que TOUT le code reste en operations supportees.
%   - Hessienne : PAS d'AD en problem-based. fmincon utilise son approximation par defaut.

clear; clc; close all;

load("./data/params.mat")            % doit definir drone et mot
p = drone;
load("./data/scenario.mat")          % cree par scenario_fixed.m
s = scen;

%% --- Parametres du NLP (memes valeurs que nlp_solv2.m) ---
Tf_init = 20;  Tf_min = 8;  Tf_max = 40;   % [s]
N       = 100;                              % intervalles
nx      = 16;  nu = 4;
g0      = 9.81;
w_h     = drone.w_h;                        % vitesse hover [rad/s]

w_min  = 0.1*w_h;   w_max = 1.8*w_h;        % [rad/s]
dw_min = -500;      dw_max = 500;           % [rad/s^2]
ang_lim = deg2rad(35);                      % limite phi, theta [rad]
c_T    = 0;                                 % poids du temps d'arrivee [-]

psi0 = 0;
x_init_bc = [scen.start_pos(:); scen.start_vel(:); 0; 0; psi0; 0; 0; 0; w_h*ones(4,1)];  % 16x1
pos_f = scen.target_pos(:);
vel_f = scen.target_vel(:);

%% --- Verification de coherence de la dynamique (numerique) ---
x_hov = [zeros(12,1); w_h*ones(4,1)];
f_hov = dyn_vec(x_hov, zeros(4,1), drone, g0);
fprintf('Hover : |f(1:12)| = %.2e | |dw/dt| = %.2e\n', norm(f_hov(1:12)), norm(f_hov(13:16)));
assert(all(isfinite(f_hov)), 'dyn_vec renvoie des NaN/Inf a l''hover');

P_ref = Pm_rows(w_h*ones(4,1), zeros(4,1), drone, mot);   % [W], hover

%% --- Variables d'optimisation (bornes incluses) ---
x_lb = [scen.x_min; scen.y_min; scen.z_min; -inf(3,1); -ang_lim; -ang_lim; -inf; -inf(3,1); w_min*ones(4,1)];
x_ub = [scen.x_max; scen.y_max; scen.z_max;  inf(3,1);  ang_lim;  ang_lim;  inf;  inf(3,1); w_max*ones(4,1)];

Tf = optimvar('Tf', 1,       'LowerBound', Tf_min,              'UpperBound', Tf_max);
X  = optimvar('X',  nx, N+1, 'LowerBound', repmat(x_lb,1,N+1),  'UpperBound', repmat(x_ub,1,N+1));
U  = optimvar('U',  nu, N+1, 'LowerBound', dw_min,              'UpperBound', dw_max);

%% --- Objectif : energie electrique, quadrature de Simpson ---
wk   = X(13:16, 1:N);     wk1  = X(13:16, 2:N+1);
dwk  = U(:, 1:N);         dwk1 = U(:, 2:N+1);
wmid  = (wk  + wk1 )/2;
dwmid = (dwk + dwk1)/2;

Pk   = Pm_rows(wk,   dwk,   drone, mot);    % 1 x N
Pmid = Pm_rows(wmid, dwmid, drone, mot);    % 1 x N
Pk1  = Pm_rows(wk1,  dwk1,  drone, mot);    % 1 x N

obj = Tf/(6*N*P_ref) * sum(Pk + 4*Pmid + Pk1);    % scalaire, adimensionne par P_ref
obj = obj + c_T*Tf;

%% --- Contraintes ---
% Collocation (trapeze, comme nlp_solv2.m) : X_{k+1} - X_k - (h/2)(f_k + f_{k+1}) = 0, h = Tf/N
F = dyn_vec(X, U, drone, g0);                                          % 16 x (N+1)
defects = X(:,2:N+1) - X(:,1:N) - (Tf/(2*N))*(F(:,1:N) + F(:,2:N+1));   % 16 x N

prob = optimproblem('Objective', obj);
prob.Constraints.colloc = (defects == 0);

% Conditions aux limites (egalites lineaires ; psi final libre)
prob.Constraints.init   = (X(:,1)       == x_init_bc);
prob.Constraints.pos_f  = (X(1:3,N+1)   == pos_f);
prob.Constraints.vel_f  = (X(4:6,N+1)   == vel_f);
prob.Constraints.att_f  = (X(7:8,N+1)   == 0);
prob.Constraints.rate_f = (X(10:12,N+1) == 0);
prob.Constraints.rot_f  = (X(13:16,N+1) == w_h*ones(4,1));

%% --- Point initial (identique a nlp_solv2.m, version sans batiment) ---
tau   = linspace(0,1,N+1);
sprof = 3*tau.^2 - 2*tau.^3;
dsdt  = (6*tau - 6*tau.^2)/Tf_init;
z_hump = 0;
p0 = scen.start_pos(:);  pf = scen.target_pos(:);

X_init = zeros(nx,N+1);
X_init(1:3,:)   = p0 + (pf-p0)*sprof;
X_init(3,:)     = X_init(3,:) + z_hump*sin(pi*tau).^2;
X_init(4:6,:)   = (pf-p0)*dsdt;
X_init(6,:)     = X_init(6,:) + z_hump*pi*sin(2*pi*tau)/Tf_init;
X_init(9,:)     = psi0;
X_init(13:16,:) = w_h*ones(4,N+1);

x0.Tf = Tf_init;
x0.X  = X_init;
x0.U  = zeros(nu,N+1);

%% Problem to structure

genDir = fullfile(pwd,'generated_fmincon');

if ~exist(genDir,'dir')
    mkdir(genDir);
end

solverProb = prob2struct(prob, x0, ...
    'ObjectiveDerivative',  'auto-reverse', ...
    'ConstraintDerivative', 'auto-reverse', ...
    'FileLocation', genDir, ...
    'ObjectiveFunctionName', 'fmincon_objective', ...
    'ConstraintFunctionName', 'fmincon_constraints');
%% --- Options fmincon ---
% Pour un test rapide de la chaine AD : MaxIterations = 3.
options = optimoptions('fmincon', ...
    'Algorithm',                'interior-point', ...
    'HessianApproximation',     'lbfgs', ...
    'SpecifyObjectiveGradient', true, ...
    'SpecifyConstraintGradient',true, ...
    'ScaleProblem',             true, ...
    'MaxIterations',            2000, ...
    'MaxFunctionEvaluations',   1e7, ...
    'OptimalityTolerance',      1e-5, ...
    'ConstraintTolerance',      1e-5, ...
    'Display',                  'iter');

solverProb.options = options;
% Hessienne : approximation par defaut de fmincon. Variante a comparer :
% options.HessianApproximation = 'lbfgs';

%% --- Resolution ---
%[sol, fval, exitflag, output] = solve(prob, x0, 'Options', options);

% Verification que les derivees ne sont PAS en differences finies :
%fprintf('Derivee objectif    : %s\n', string(output.objectivederivative));
%fprintf('Derivee contraintes : %s\n', string(output.constraintderivative));

[z_sol, fval, exitflag, output, lambda, grad, hessian] = ...
    fmincon(solverProb);

%% --- Extraction (memes noms que nlp_solv2.m, pour reutiliser nlp_simu2.m) ---
%Tf_sol = sol.Tf;
%X_sol  = sol.X;                         % 16 x (N+1)
%U_sol  = sol.U;                         % 4 x (N+1), accelerations [rad/s^2]

idx = varindex(prob);

Tf_sol = z_sol(idx.Tf);
X_sol  = reshape(z_sol(idx.X), nx, N+1);
U_sol  = reshape(z_sol(idx.U), nu, N+1);

t_sol  = linspace(0, Tf_sol, N+1);


% Energie electrique [J] (U_sol = accelerations, vitesses dans X_sol(13:16,:))
Wn  = X_sol(13:16,:);
E_P = Pm_rows(Wn(:,1:N),               U_sol(:,1:N),               drone, mot) ...
    + 4*Pm_rows((Wn(:,1:N)+Wn(:,2:N+1))/2, (U_sol(:,1:N)+U_sol(:,2:N+1))/2, drone, mot) ...
    + Pm_rows(Wn(:,2:N+1),             U_sol(:,2:N+1),             drone, mot);
E_sol = Tf_sol/(6*N) * sum(E_P);

fprintf('Tf optimal : %.2f s\n', Tf_sol);
fprintf('Energie    : %.1f J  (hover pendant Tf : %.1f J)\n', E_sol, P_ref*Tf_sol);
fprintf('exitflag = %d | iterations = %d | viol. contraintes = %.2e\n', ...
        exitflag, output.iterations, output.constrviolation);

save('sols/sol_fmincon.mat', 'X_sol', 'U_sol', 't_sol', 'Tf_sol');

%% ======================= Fonctions locales =======================

function F = dyn_vec(Xm, Um, drone, g0)
% Dynamique vectorisee sur M noeuds (memes equations que f_dyn de nlp_solv2.m, sans Jr).
% Entrees : Xm 16 x M, Um 4 x M (numeriques OU expressions d'optimisation)
% Sortie  : F  16 x M. Operations elementaires uniquement (compatibles AD de solve).
    vx = Xm(4,:);  vy = Xm(5,:);  vz = Xm(6,:);
    phi = Xm(7,:); th = Xm(8,:);  psi = Xm(9,:);
    pp = Xm(10,:); q = Xm(11,:);  r = Xm(12,:);
    w1 = Xm(13,:); w2 = Xm(14,:); w3 = Xm(15,:); w4 = Xm(16,:);

    cphi = cos(phi); sphi = sin(phi);
    cth  = cos(th);  sth  = sin(th);  tth = sth./cth;
    cpsi = cos(psi); spsi = sin(psi);

    % Poussee et couples
    T    = drone.k_T*(w1.^2 + w2.^2 + w3.^2 + w4.^2);
    Gphi = drone.l*drone.k_T*(w4.^2 - w2.^2);
    Gth  = drone.l*drone.k_T*(w3.^2 - w1.^2);
    Gpsi = drone.k_Q*(w1.^2 - w2.^2 + w3.^2 - w4.^2);

    % Translation : (R*[0;0;T] - [0;0;m g])/m  (3e colonne de R)
    ax = (cpsi.*sth.*cphi + spsi.*sphi).*T/drone.m;
    ay = (spsi.*sth.*cphi - cpsi.*sphi).*T/drone.m;
    az = (cth.*cphi).*T/drone.m - g0;

    % Cinematique d'Euler : deta = W(eta)*[p;q;r]
    dphi = pp + sphi.*tth.*q + cphi.*tth.*r;
    dth  = cphi.*q - sphi.*r;
    dpsi = (sphi.*q + cphi.*r)./cth;

    % Euler (J diagonale) : J*dom = -om x (J om) + G
    dp = (-(drone.Jzz - drone.Jyy).*q.*r  + Gphi)/drone.Jxx;
    dq = (-(drone.Jxx - drone.Jzz).*r.*pp + Gth )/drone.Jyy;
    dr = (-(drone.Jyy - drone.Jxx).*pp.*q + Gpsi)/drone.Jzz;

    F = [vx; vy; vz; ax; ay; az; dphi; dth; dpsi; dp; dq; dr; Um];
end

function P = Pm_rows(W, dW, drone, mot)
% Puissance electrique moteurs [W] par noeud : somme sur les 4 rotors.
% Entrees : W, dW 4 x M ; Sortie : 1 x M.
    a  = mot.Jr*dW + drone.k_Q*W.^2;
    Pr = (mot.R/mot.KE^2)*a.^2 + W.*a;
    P  = Pr(1,:) + Pr(2,:) + Pr(3,:) + Pr(4,:);
end