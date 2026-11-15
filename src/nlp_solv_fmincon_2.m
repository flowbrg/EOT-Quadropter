%% Optimisation de trajectoire - Quadrirotor 3D, energie minimale, AVEC obstacles
%  Optimization Toolbox, approche problem-based (optimvar + prob2struct -> fmincon)
%
%  Modele d'etat 16 etats : [x y z vx vy vz phi theta psi p q r w1 w2 w3 w4]
%  Commandes              : U = [dw1..dw4]  accelerations angulaires [rad/s^2]
%
%  Equivalent de nlp_solv_casadi_2.m (CasADi/IPOPT), avec batiments
%  (super-ellipsoides d'exposant pair p_sq), et construit comme nlp_solv_fmincon.m.
%  Prerequis : Optimization Toolbox ; params.mat (drone, mot) ; scenario.mat (scen).
%
%  Derivees :
%   - gradient de l'objectif et Jacobiennes des contraintes : AD (auto-reverse)
%     via prob2struct, tant que TOUT le code reste en operations supportees.
%   - Hessienne : L-BFGS (interior-point).

clear; clc; close all;

load("./data/params.mat")            % doit definir drone et mot
load("./data/scenario.mat")          % cree par scenario_fixed.m

%% --- Parametres du NLP (memes valeurs que nlp_solv_casadi_2.m) ---
Tf_init = 20;  Tf_min = 8;  Tf_max = 40;   % [s]
N       = 100;                              % intervalles
nx      = 16;  nu = 4;
g0      = 9.81;
w_h     = drone.w_h;                        % vitesse hover [rad/s]

w_min  = 0.1*w_h;   w_max = 1.8*w_h;        % [rad/s]
dw_min = -500;      dw_max = 500;           % [rad/s^2]
ang_lim = deg2rad(35);                      % limite phi, theta [rad]
c_T    = 0;                                 % poids du temps d'arrivee [-]

% Super-ellipsoide d'exposant pair p_sq (plus grand -> plus proche du pave,
% mais NLP moins bien conditionne)
p_sq = 4;

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
% Collocation (trapeze) : X_{k+1} - X_k - (h/2)(f_k + f_{k+1}) = 0, h = Tf/N
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

% Obstacles (batiments AABB, formulation super-ellipsoide lisse) :
% s_b(r) = ((x-xc)/ax)^p + ((y-yc)/ay)^p + (z/az)^p >= 1
% avec ax = dx+ecart, ay = dy+ecart, az = hz+ecart (z >= 0 : centre au sol).
% p_sq pair -> la puissance entiere est identique a ((.)^2)^(p/2) du script CasADi.
% Une contrainte vectorielle (1 x N+1) par batiment, sur tous les noeuds.
for b = 1:scen.n_buildings
    xc = scen.buildings(b,1);  yc = scen.buildings(b,2);
    ax = scen.buildings(b,3) + scen.ecart;
    ay = scen.buildings(b,4) + scen.ecart;
    az = scen.buildings(b,5) + scen.ecart;

    sb = ((X(1,:) - xc)/ax).^p_sq ...
       + ((X(2,:) - yc)/ay).^p_sq ...
       + ( X(3,:)      /az).^p_sq;

    prob.Constraints.(sprintf('obst_%d', b)) = (sb >= 1);
end

%% --- Point initial (identique a nlp_solv_casadi_2.m : plateau d'altitude) ---
% Position : smoothstep xy (vitesse nulle aux extremites) + profil d'altitude
% en "plateau" : montee rapide, croisiere au-dessus du batiment le plus haut,
% descente finale.
tau   = linspace(0, 1, N+1);
sprof = 3*tau.^2 - 2*tau.^3;
dsdt  = (6*tau - 6*tau.^2) / Tf_init;

z_cruise = max(scen.buildings(:,5)) + 2*scen.ecart;   % [m]
if z_cruise >= scen.z_max
    error('z_cruise (%.1f m) >= z_max (%.1f m)', z_cruise, scen.z_max);
end
sstep  = @(a) 3*min(max(a,0),1).^2 - 2*min(max(a,0),1).^3;
z_prof = z_cruise*(sstep(tau/0.15) - sstep((tau-0.85)/0.15));

p0 = scen.start_pos(:);  pf = scen.target_pos(:);

X_init = zeros(nx,N+1);
X_init(1:3,:)   = p0 + (pf-p0)*sprof;
X_init(3,:)     = X_init(3,:) + z_prof;
X_init(4:6,:)   = (pf-p0)*dsdt;
X_init(6,:)     = X_init(6,:) + gradient(z_prof, Tf_init/N);
X_init(9,:)     = psi0;
X_init(13:16,:) = w_h*ones(4,N+1);

x0.Tf = Tf_init;
x0.X  = X_init;
x0.U  = zeros(nu,N+1);

%% --- Problem to structure ---
genDir = fullfile(pwd,'generated_fmincon_2');
if ~exist(genDir,'dir')
    mkdir(genDir);
end

solverProb = prob2struct(prob, x0, ...
    'ObjectiveDerivative',  'auto-reverse', ...
    'ConstraintDerivative', 'auto-reverse', ...
    'FileLocation', genDir, ...
    'ObjectiveFunctionName', 'fmincon_objective_2', ...
    'ConstraintFunctionName', 'fmincon_constraints_2');

%% --- Options fmincon ---
% Pour un test rapide de la chaine AD : MaxIterations = 3.
options = optimoptions('fmincon', ...
    'Algorithm',                'interior-point', ...
    'HessianApproximation',     'lbfgs', ...
    'SpecifyObjectiveGradient', true, ...
    'SpecifyConstraintGradient',true, ...
    'ScaleProblem',             true, ...
    'MaxIterations',            3000, ...
    'MaxFunctionEvaluations',   1e7, ...
    'OptimalityTolerance',      1e-5, ...
    'ConstraintTolerance',      1e-5, ...
    'Display',                  'iter');

solverProb.options = options;

%% --- Resolution ---
[z_sol, fval, exitflag, output, lambda, grad, hessian] = ...
    fmincon(solverProb);

%% --- Extraction (memes noms que nlp_solv_casadi_2.m / nlp_solv_fmincon.m) ---
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

% Verification a posteriori des obstacles : min_k s_b(r_k) doit etre >= 1
for b = 1:scen.n_buildings
    xc = scen.buildings(b,1);  yc = scen.buildings(b,2);
    ax = scen.buildings(b,3) + scen.ecart;
    ay = scen.buildings(b,4) + scen.ecart;
    az = scen.buildings(b,5) + scen.ecart;
    sb_sol = ((X_sol(1,:)-xc)/ax).^p_sq + ((X_sol(2,:)-yc)/ay).^p_sq + (X_sol(3,:)/az).^p_sq;
    fprintf('Batiment %d : min s_b = %.4f (>= 1 requis)\n', b, min(sb_sol));
end

fprintf('Tf optimal : %.2f s\n', Tf_sol);
fprintf('Energie    : %.1f J  (hover pendant Tf : %.1f J)\n', E_sol, P_ref*Tf_sol);
fprintf('exitflag = %d | iterations = %d | viol. contraintes = %.2e\n', ...
        exitflag, output.iterations, output.constrviolation);

save('sols/sol_fmincon_2.mat', 'X_sol', 'U_sol', 't_sol', 'Tf_sol', 'E_sol');

%% ======================= Fonctions locales =======================

function F = dyn_vec(Xm, Um, drone, g0)
% Dynamique vectorisee sur M noeuds (memes equations que f_dyn de nlp_solv_casadi_2.m, sans Jr).
% Entrees : Xm 16 x M, Um 4 x M (numeriques OU expressions d'optimisation)
% Sortie  : F  16 x M. Operations elementaires uniquement (compatibles AD).
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