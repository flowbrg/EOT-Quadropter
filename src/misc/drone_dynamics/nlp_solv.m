%% Optimisation de trajectoire — Quadrirotor 3D, energie minimale
%  CasADi + IPOPT
% Modele d'etat 12 etats : [x y z vx vy vz phi theta psi p q r]
% Commandes : 4 vitesses d'helices [w1..w4], normalisees par w_hover
% Trajectoire sans obstacle

%clear; clc; close all;
import casadi.*

load("params.mat")
p = drone;
load("scenario.mat")          % cree par scenario_fixed.m
s = scen;



%% --- Paramètres du NLP ---
Tf_init  = 20;    % initialisation Tf (s)
Tf_min   = 8;     % [s]
Tf_max   = 40;    % [s]
N        = 100;    % nombre d'intervalles

g0       = 9.81;
w_h      = drone.w_h;   % vitesse hover analytique [rad/s]

% Bornes commandes (HYPOTHESES : a valider)
w_min = 0.1*w_h;                 % [rad/s]
w_max = 1.8*w_h;                 % [rad/s] -> rapport poussee/poids = 4*1.8^2/4 = 3.24
% Rotor angular acceleration bounds [rad/s^2]
dw_min = -500;
dw_max = 500;

% Limite d'attitude (evite la singularite d'Euler en theta = +-pi/2)
ang_lim  = deg2rad(35);              % (HYPOTHESE : a valider)

% Ponderations du critere
%W    = 1e-2*eye(4);   % regularisation sur les variations de commande
c_T  = 0;             % poids du temps d'arrivee (extension "advanced") [-]

% Conditions aux limites
psi0 = 0;
x_init_bc = [ ...
    s.start_pos(:);
    s.start_vel(:);
    0; 0; psi0;
    0; 0; 0;
    w_h*ones(4,1)];
pos_f = s.target_pos(:);
vel_f = s.target_vel(:);

nx = 16;    % State size
nu = 4;     % Input size

%% --- Déclaration des variables symboliques CasADi ---

% Tf libre
Tf = MX.sym('Tf');
h  = Tf / N;

% États : [x y z vx vy vz phi theta psi p q r w1 w2 w3 w4]  taille 16 x (N+1)
X  = MX.sym('X', nx, N+1);

% Commandes : [w1..w4]/w_hover  taille 4 x (N+1)
U  = MX.sym('U', nu, N+1);

%% --- Fonction de dynamique ---
% Entrée : état x (12x1), commande normalisee uc (4x1)
% Sortie : xdot (12x1)
% Meme equations que dyn.m (sans Jr). J est diagonale : on evite J\(.)
% avec des MX et on divise elementwise. Verification numerique plus bas.

x_s   = MX.sym('x_s',  nx);
uc_s  = MX.sym('uc_s', nu); % = dw_s

v_s   = x_s(4:6);
phi_s = x_s(7); theta_s = x_s(8); psi_s = x_s(9);
om_s  = x_s(10:12);   % [p; q; r]
w_s   = x_s(13:16);


% Intergrator w
dw_s = uc_s;

cphi = cos(phi_s);   sphi = sin(phi_s);
cth  = cos(theta_s); sth  = sin(theta_s); tth = sth/cth;
cpsi = cos(psi_s);   spsi = sin(psi_s);

% Poussee et couples
w_s2  = w_s.^2;
T_s     = p.k_T*sum(w_s2);
Gphi_s  = p.l*p.k_T*(w_s2(4) - w_s2(2));
Gth_s   = p.l*p.k_T*(w_s2(3) - w_s2(1));
Gpsi_s  = p.k_Q*(w_s2(1) - w_s2(2) + w_s2(3) - w_s2(4));

% Matrice de rotation corps -> inertiel (ZYX)
R_s = [cpsi*cth,  cpsi*sth*sphi - spsi*cphi,  cpsi*sth*cphi + spsi*sphi;
       spsi*cth,  spsi*sth*sphi + cpsi*cphi,  spsi*sth*cphi - cpsi*sphi;
       -sth,      cth*sphi,                   cth*cphi];

% Relation vitesses angulaires corps -> derivee des angles d'Euler
Wm_s = [1, sphi*tth,  cphi*tth;
        0, cphi,     -sphi;
        0, sphi/cth,  cphi/cth];
%Wm_s = eye(3);

% Translation
a_s = (R_s*vertcat(0, 0, T_s) - [0; 0; p.m*g0]) / p.m;

% Rotation : J*dw = -w x (J w) + G
p_ = om_s(1); q_ = om_s(2); r_ = om_s(3);
Jw     = [p.Jxx*p_; p.Jyy*q_; p.Jzz*r_];
wxJw   = [q_*Jw(3) - r_*Jw(2);
          r_*Jw(1) - p_*Jw(3);
          p_*Jw(2) - q_*Jw(1)];
dom_s   = (-wxJw + [Gphi_s; Gth_s; Gpsi_s]) ./ [p.Jxx; p.Jyy; p.Jzz];

deta_s = Wm_s*om_s;

f_dyn = Function('f_dyn', {x_s, uc_s}, { ...
    vertcat(v_s, a_s, deta_s, dom_s, dw_s) });

%% --- Verification de coherence ---
% En hover (moteurs a w_h, commande = 1) : les 12 premieres derivees ~ 0.
% SoC et z ne sont pas forcement nuls (decharge, integrateurs).
x_hov  = [zeros(12,1); w_h*ones(4,1)];
f_hov  = full(f_dyn(x_hov, zeros(4,1)));
fprintf('Hover : |f(1:12)| = %.2e | |dw/dt| = %.2e\n', ...
        norm(f_hov(1:12)), norm(f_hov(13:16)));
assert(all(isfinite(f_hov)), 'f_dyn renvoie des NaN/Inf a l''hover');
%% --- Construction du NLP ---

J    = 0;           % critère
g    = {};          % contraintes d'égalité (collocation)
g_lb = {};          % bornes inférieures
g_ub = {};          % bornes supérieures

% Pour un indice i, la valeur de g(i) doit etre comprise entre
% g_lb(i) et g_ub(i)

% P_m(omega, omega_dot) [W], vectors 4x1 in rad/s and rad/s^2
Pm = @(w,dw) sum( ...
    (mot.R./(mot.KE.^2)) .* ...
    (mot.Jr.*dw + p.k_Q.*w.^2).^2 ...
    + w.*(mot.Jr.*dw + p.k_Q.*w.^2) );

P_ref = full(Pm(w_h*ones(4,1), zeros(4,1)));

for k = 1:N
    xk = X(:,k);  xk1 = X(:,k+1);
    uk = U(:,k);  uk1 = U(:,k+1);

    wk  = xk(13:16);
    wk1 = xk1(13:16);
    
    dwk  = uk;
    dwk1 = uk1;
    
    wmid  = (wk + wk1)/2;
    dwmid = (dwk + dwk1)/2;
    
    % Simpson quadrature of electrical energy
    J = J + h/(6*P_ref) * ( ...
    Pm(wk,   dwk) + ...
    4*Pm(wmid,dwmid) + ...
    Pm(wk1,  dwk1) );

    fk  = f_dyn(xk,uk);
    fk1 = f_dyn(xk1,uk1);

    col = xk1 - xk - (h/2)*(fk + fk1);
    g{end+1} = col;
    g_lb{end+1} = zeros(nx,1);  g_ub{end+1} = zeros(nx,1);
end

% Terme de temps d'arrivee (nul par defaut)
J = J + c_T*Tf;

% (Contraintes d'obstacles : desactivees dans cette etape, scenario sans batiment)

%% Conditions aux limites (égalités)
% État initial (16 egalites)
g{end+1}    = X(:,1) - x_init_bc;
g_lb{end+1} = zeros(nx,1);
g_ub{end+1} = zeros(nx,1);

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

g{end+1} = X(13:16,N+1) - w_h*ones(4,1);
g_lb{end+1} = zeros(4,1);
g_ub{end+1} = zeros(4,1);

% Assemblage vecteur de décision
w_vec    = vertcat(Tf, vec(X), vec(U));
g_vec    = vertcat(g{:});
g_lb_vec = vertcat(g_lb{:});
g_ub_vec = vertcat(g_ub{:});

%% Bornes sur les variables de décision

% Tf
w_lb = Tf_min;
w_ub = Tf_max;

% States:
% [x y z vx vy vz phi theta psi p q r w1 w2 w3 w4]

x_lb = [s.x_min; s.y_min; s.z_min; -inf(3,1);
        -ang_lim; -ang_lim; -inf; -inf(3,1); w_min*ones(4,1)];

x_ub = [s.x_max; s.y_max;  s.z_max; inf(3,1);
        ang_lim; ang_lim; inf; inf(3,1); w_max*ones(4,1)];

w_lb = [w_lb; repmat(x_lb,N+1,1)];
w_ub = [w_ub; repmat(x_ub,N+1,1)];

% Commandes U
u_lb = dw_min*ones(nu,1);
u_ub = dw_max*ones(nu,1);

w_lb = [w_lb; repmat(u_lb,N+1,1)];
w_ub = [w_ub; repmat(u_ub,N+1,1)];

%% Initial guess

tau   = linspace(0,1,N+1);
sprof = 3*tau.^2 - 2*tau.^3;
dsdt  = (6*tau - 6*tau.^2)/Tf_init;

z_hump = 5;

p0 = s.start_pos(:);
pf = s.target_pos(:);

X_init = zeros(nx,N+1);

% Position
X_init(1:3,:) = p0 + (pf-p0)*sprof;
X_init(3,:) = X_init(3,:) + z_hump*sin(pi*tau).^2;

% Velocity
X_init(4:6,:) = (pf-p0)*dsdt;
X_init(6,:) = X_init(6,:) + ...
    z_hump*pi*sin(2*pi*tau)/Tf_init;

% Initial yaw
X_init(9,:) = psi0;

% Rotor speeds: start with hover
X_init(13:16,:) = w_h*ones(4,N+1);

% Initial input: zero angular acceleration
u_init = zeros(nu,N+1);

w0 = [Tf_init;
      reshape(X_init,[],1);
      reshape(u_init,[],1)];

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
w_sol = full(sol.x);

Tf_sol = w_sol(1);

nX = nx*(N+1);

idx_X = 2:(nX+1);
idx_U = (nX+2):length(w_sol);

X_sol = reshape(w_sol(idx_X),nx,N+1);
U_sol = reshape(w_sol(idx_U),nu,N+1);

t_sol = linspace(0,Tf_sol,N+1);

% Extract rotor speeds and accelerations
W_sol  = X_sol(13:16,:);
DW_sol = U_sol;

% Energie mecanique d'helice : E = int k_Q * sum(Omega_i^3) dt
P_sol  = p.k_Q * w_h^3 * sum(U_sol.^3, 1);  % [W], par intervalle
E_sol = 0;              % [J]
for k = 1:N
    hk  = t_sol(k+1) - t_sol(k);
    om0 = U_sol(:,k)*w_h;  om1 = U_sol(:,k+1)*w_h;
    dom = (om1-om0)/hk;    omm = (om0+om1)/2;
    E_sol = E_sol + hk*(Pm(om0,dom) + 4*Pm(omm,dom) + Pm(om1,dom))/6;
end

fprintf('Tf optimal : %.2f s\n', Tf_sol);
fprintf('Energie    : %.1f J  (hover pendant Tf : %.1f J)\n', E_sol, P_ref*Tf_sol);
fprintf('Statut IPOPT : %s\n', solver.stats().return_status);