%% Optimisation de trajectoire — Quadrirotor 3D, energie minimale
%  CasADi + IPOPT
% Modele d'etat 12 etats : [x y z vx vy vz phi theta psi p q r]
% Commandes : 4 vitesses d'helices [w1..w4], normalisees par w_hover
% Trajectoire avec obstacle

%clear; clc; close all;
import casadi.*

load("data/params.mat")
p = drone;
load("data/scenario.mat")          % cree par scenario_fixed.m
s = scen;
addpath("./model")


%% --- Paramètres du NLP ---
Tf_init  = 20;    % initialisation Tf (s)
Tf_min   = 8;     % [s]  (HYPOTHESE : a valider)
Tf_max   = 40;    % [s]  (HYPOTHESE : a valider)
N        = 60;    % nombre d'intervalles

g0       = 9.81;
w_h      = sqrt(p.m*g0/(4*p.k_T));   % vitesse hover analytique [rad/s]
P_h      = 4*p.k_Q*w_h^3;            % puissance hover [W] (normalisation du critere)

% Bornes commandes (HYPOTHESES : a valider)
Omega_min = 0.1*w_h;                 % [rad/s]
Omega_max = 1.8*w_h;                 % [rad/s] -> rapport poussee/poids = 4*1.8^2/4 = 3.24
u_min     = Omega_min/w_h;           % commande normalisee
u_max     = Omega_max/w_h;

% Limite d'attitude (evite la singularite d'Euler en theta = +-pi/2)
ang_lim  = deg2rad(35);              % (HYPOTHESE : a valider)

% Ponderations du critere
W    = 1e-2*eye(4);   % regularisation sur les variations de commande
c_T  = 0;             % poids du temps d'arrivee (extension "advanced") [-]

% Conditions aux limites
psi0 = 0;
x_init_bc = [s.start_pos(:);  s.start_vel(:);  0; 0; psi0; 0; 0; 0];
pos_f = s.target_pos(:);
vel_f = s.target_vel(:);

% Contraintes batiments : super-ellipsoide d'exposant pair p_sq (p_sq = 8)
% p_sq plus grand -> forme plus proche du pave, mais NLP moins bien conditionne
p_sq = 8;

%% --- Déclaration des variables symboliques CasADi ---

% Tf libre
Tf = MX.sym('Tf');
h  = Tf / N;

% États : [x y z vx vy vz phi theta psi p q r]  taille 12 x (N+1)
X  = MX.sym('X', 12, N+1);

% Commandes : [w1..w4]/w_hover  taille 4 x N
U  = MX.sym('U', 4, N);

%% --- Fonction de dynamique ---
% Entrée : état x (12x1), commande normalisee uc (4x1)
% Sortie : xdot (12x1)
% Meme equations que dyn.m (sans Jr). J est diagonale : on evite J\(.)
% avec des MX et on divise elementwise. Verification numerique plus bas.

x_s   = MX.sym('x_s',  12);
uc_s  = MX.sym('uc_s', 4);

v_s   = x_s(4:6);
phi_s = x_s(7); theta_s = x_s(8); psi_s = x_s(9);
om_s  = x_s(10:12);   % [p; q; r]

cphi = cos(phi_s);   sphi = sin(phi_s);
cth  = cos(theta_s); sth  = sin(theta_s); tth = sth/cth;
cpsi = cos(psi_s);   spsi = sin(psi_s);

Om2 = (uc_s*w_h).^2;                      % Omega_i^2

% Poussee et couples
T_s     = p.k_T*sum(Om2);
Gphi_s  = p.l*p.k_T*(Om2(4) - Om2(2));
Gth_s   = p.l*p.k_T*(Om2(3) - Om2(1));
Gpsi_s  = p.k_Q*(Om2(1) - Om2(2) + Om2(3) - Om2(4));

% Matrice de rotation corps -> inertiel (ZYX)
R_s = [cpsi*cth,  cpsi*sth*sphi - spsi*cphi,  cpsi*sth*cphi + spsi*sphi;
       spsi*cth,  spsi*sth*sphi + cpsi*cphi,  spsi*sth*cphi - cpsi*sphi;
       -sth,      cth*sphi,                   cth*cphi];

% Relation vitesses angulaires corps -> derivee des angles d'Euler
Wm_s = [1, sphi*tth,  cphi*tth;
        0, cphi,     -sphi;
        0, sphi/cth,  cphi/cth];

% Translation
a_s = (R_s*vertcat(0, 0, T_s) - [0; 0; p.m*g0]) / p.m;

% Rotation : J*dw = -w x (J w) + G
p_ = om_s(1); q_ = om_s(2); r_ = om_s(3);
Jw     = [p.Jxx*p_; p.Jyy*q_; p.Jzz*r_];
wxJw   = [q_*Jw(3) - r_*Jw(2);
          r_*Jw(1) - p_*Jw(3);
          p_*Jw(2) - q_*Jw(1)];
dw_s   = (-wxJw + [Gphi_s; Gth_s; Gpsi_s]) ./ [p.Jxx; p.Jyy; p.Jzz];

deta_s = Wm_s*om_s;

f_dyn = Function('f_dyn', {x_s, uc_s}, { ...
    vertcat(v_s, a_s, deta_s, dw_s) });

%% --- Verification de coherence avec dyn.m (source de verite) ---
rng(0);
x_chk = [randn(3,1); randn(3,1); 0.3*randn(3,1); 0.5*randn(3,1)];
u_chk = 1 + 0.2*rand(4,1);
err_chk = norm(full(f_dyn(x_chk, u_chk)) - dyn(x_chk, u_chk*w_h, p));
assert(err_chk < 1e-9, 'f_dyn (CasADi) incoherent avec dyn.m : %.2e', err_chk);
fprintf('Verif f_dyn vs dyn.m : ecart = %.2e | hover |f| = %.2e\n', ...
        err_chk, norm(full(f_dyn(zeros(12,1), ones(4,1)))));

%% --- Construction du NLP ---

J    = 0;           % critère
g    = {};          % contraintes d'égalité (collocation)
g_lb = {};          % bornes inférieures
g_ub = {};          % bornes supérieures

% Pour un indice i, la valeur de g(i) doit etre comprise entre
% g_lb(i) et g_ub(i)

for k = 1:N
    xk  = X(:, k);
    xk1 = X(:, k+1);
    uk  = U(:, k);

    % Critère : energie (sum Omega^3, normalisee par P_h -> [s de hover])
    %   P_i = k_Q*Omega_i^3 ; u constante par intervalle => integrale exacte
    J = J + (sum(uk.^3)/4) * h;

    % Regularisation sur les variations de commande
    if k > 1
        du = uk - U(:, k-1);
        J  = J + du'*W*du;
    end

    % Collocation trapézoïdale
    fk  = f_dyn(xk,  uk);
    fk1 = f_dyn(xk1, uk);
    col = xk1 - xk - (h/2) * (fk + fk1);

    g{end+1}    = col;
    g_lb{end+1} = zeros(12,1);
    g_ub{end+1} = zeros(12,1);
end

% Terme de temps d'arrivee (nul par defaut)
J = J + c_T*Tf;

% Contraintes d'obstacles (batiments AABB, formulation super-ellipsoide lisse)
% s_b(r) = ((x-xc)/ax)^p + ((y-yc)/ay)^p + (z/az)^p >= 1
% avec ax = dx+ecart, ay = dy+ecart, az = hz+ecart  (z >= 0 : centre au sol)
for k = 1:N+1
    xk = X(:,k);
    for b = 1:s.n_buildings
        xc = s.buildings(b,1);  yc = s.buildings(b,2);
        ax = s.buildings(b,3) + s.ecart;
        ay = s.buildings(b,4) + s.ecart;
        az = s.buildings(b,5) + s.ecart;

        sb = (((xk(1)-xc)/ax)^2)^(p_sq/2) ...
           + (((xk(2)-yc)/ay)^2)^(p_sq/2) ...
           + (((xk(3)   )/az)^2)^(p_sq/2);

        g{end+1}    = sb;
        g_lb{end+1} = 1;
        g_ub{end+1} = inf;
    end
end

%% Conditions aux limites (égalités)
% État initial (12 egalites)
g{end+1}    = X(:,1) - x_init_bc;
g_lb{end+1} = zeros(12,1);
g_ub{end+1} = zeros(12,1);

% État final : position, vitesse, phi, theta, p q r (psi final libre)
g{end+1}    = X(1:3, N+1) - pos_f;
g_lb{end+1} = zeros(3,1);
g_ub{end+1} = zeros(3,1);

g{end+1}    = X(4:6, N+1) - vel_f;
g_lb{end+1} = zeros(3,1);
g_ub{end+1} = zeros(3,1);

g{end+1}    = X(7:8, N+1);
g_lb{end+1} = zeros(2,1);
g_ub{end+1} = zeros(2,1);

g{end+1}    = X(10:12, N+1);
g_lb{end+1} = zeros(3,1);
g_ub{end+1} = zeros(3,1);

% Assemblage vecteur de décision
w_vec    = vertcat(Tf, vec(X), vec(U));
g_vec    = vertcat(g{:});
g_lb_vec = vertcat(g_lb{:});
g_ub_vec = vertcat(g_ub{:});

%% Bornes sur les variables de décision

% Tf
w_lb = Tf_min;
w_ub = Tf_max;

% États X : [x y z vx vy vz phi theta psi p q r] x (N+1)
x_lb = [s.x_min; s.y_min; s.z_min; -inf(3,1); -ang_lim; -ang_lim; -inf; -inf(3,1)];
x_ub = [s.x_max; s.y_max; s.z_max;  inf(3,1);  ang_lim;  ang_lim;  inf;  inf(3,1)];
w_lb = [w_lb; repmat(x_lb, N+1, 1)];
w_ub = [w_ub; repmat(x_ub, N+1, 1)];

% Commandes U : [w1..w4]/w_h x N
u_lb = u_min*ones(4,1);
u_ub = u_max*ones(4,1);
w_lb = [w_lb; repmat(u_lb, N, 1)];
w_ub = [w_ub; repmat(u_ub, N, 1)];

%% Point initial
% Profil de position "smoothstep" (vitesse nulle aux extremites) + bosse
%% Point initial
% Position : smoothstep xy (vitesse nulle aux extremites) + profil d'altitude
% en "plateau" : montee rapide, croisiere au-dessus du batiment le plus haut,
% descente finale. (L'ancienne bosse z_hump*sin^2 traversait B1..B4.)
tau   = linspace(0, 1, N+1);
sprof = 3*tau.^2 - 2*tau.^3;
dsdt  = (6*tau - 6*tau.^2) / Tf_init;

z_cruise = max(s.buildings(:,5)) + 2*s.ecart;   % [m] ici 17 m < z_max = 30
if z_cruise >= s.z_max
    error('z_cruise (%.1f m) >= z_max (%.1f m)', z_cruise, s.z_max);
end
sstep  = @(a) 3*min(max(a,0),1).^2 - 2*min(max(a,0),1).^3;
z_prof = z_cruise*(sstep(tau/0.15) - sstep((tau-0.85)/0.15));

p0 = s.start_pos(:); pf = s.target_pos(:);
X_init = zeros(12, N+1);
X_init(1:3,:) = p0 + (pf - p0)*sprof;
X_init(3,:)   = X_init(3,:) + z_prof;
X_init(4:6,:) = (pf - p0)*dsdt;
X_init(6,:)   = X_init(6,:) + gradient(z_prof, Tf_init/N);
X_init(9,:)   = psi0;

u_init = ones(4, N);                         % hover : Omega_i = w_h

w0 = [Tf_init; reshape(X_init, [], 1); reshape(u_init, [], 1)];

%% --- Solveur IPOPT ---
nlp  = struct('x', w_vec, 'f', J, 'g', g_vec);
opts = struct();
opts.ipopt.max_iter             = 2000;
opts.ipopt.tol                  = 1e-5;
opts.ipopt.constr_viol_tol      = 1e-5;

opts.ipopt.nlp_scaling_method   = 'gradient-based';

% Stratégie de barrière
opts.ipopt.mu_strategy          = 'adaptive';
opts.ipopt.mu_init              = 1e-1;

% Solveur linéaire
opts.ipopt.linear_solver        = 'mumps';    % remplacer par 'ma57' si HSL dispo
opts.ipopt.mumps_pivtol         = 1e-4;

% Hessien exact par défaut (AD CasADi). NLP ici : 1 + 12(N+1) + 4N variables.
% opts.ipopt.hessian_approximation = 'limited-memory';

opts.ipopt.print_level          = 5;

solver = nlpsol('solver', 'ipopt', nlp, opts);

sol = solver('x0',  w0, ...
             'lbx', w_lb, 'ubx', w_ub, ...
             'lbg', g_lb_vec, 'ubg', g_ub_vec);

%% --- Extraction de la solution ---
w_sol  = full(sol.x);

Tf_sol = w_sol(1);
idx_X  = 2 : 12*(N+1)+1;
idx_U  = 12*(N+1)+2 : length(w_sol);

X_sol  = reshape(w_sol(idx_X), 12, N+1);
U_sol  = reshape(w_sol(idx_U), 4, N);      % normalisee par w_h
t_sol  = linspace(0, Tf_sol, N+1);

% Energie mecanique d'helice : E = int k_Q * sum(Omega_i^3) dt
P_sol  = p.k_Q * w_h^3 * sum(U_sol.^3, 1);  % [W], par intervalle
E_sol  = sum(P_sol .* diff(t_sol));         % [J]

fprintf('Tf optimal : %.2f s\n', Tf_sol);
fprintf('Energie    : %.1f J  (hover pendant Tf : %.1f J)\n', E_sol, P_h*Tf_sol);
fprintf('Statut IPOPT : %s\n', solver.stats().return_status);