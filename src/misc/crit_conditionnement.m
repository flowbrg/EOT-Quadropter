%% crit_conditionnement.m
% Conditionnement du critère à minimiser
clc; clear; close all;

load("../data/params.mat")

w_h      = sqrt(drone.m*9.81/(4*drone.k_T))  % hover [rad/s]
w_max    = 2*w_h;                   % hypothèse : a remplacer

% --- Coefficients kappa (Éq. 31), ordre [w^0 ... w^4, wdot^2] ---
A = mot.R/mot.KE^2;
kap = [ A*mot.Tf^2;
    2*A*mot.Df*mot.Tf + mot.Tf;
    A*(mot.Df^2 + 2*mot.d*mot.Tf) + mot.Df;
    2*A*mot.d*mot.Df + mot.d;
    A*mot.d^2;
    A*mot.Jm^2 ];
disp(table((1:6)', kap, 'VariableNames', {'i','kappa_i'}))

% --- Conditionnement de la base {1,w,...,w^4} sur [0,w_max] ---
w  = linspace(0, w_max, 200)';
V  = w.^(0:4);                       % [200x5] Vandermonde brut
Vn = (w/w_max).^(0:4);               % [200x5] variable normalisee
fprintf('cond(V)  brut     = %.3e\n', cond(V));
fprintf('cond(Vn) normalise = %.3e\n', cond(Vn));

% --- Dynamique des coefficients (echelle w_max) ---
kn = kap(1:5) .* w_max.^(0:4)';      % [5x1] contributions a w = w_max [W]
fprintf('max/min |kappa_i w_max^i| = %.3e\n', max(abs(kn(kn~=0)))/min(abs(kn(kn~=0))));

% --- Contribution de chaque terme au hover [W] ---
Ph = kap(1:5) .* w_h.^(0:4)';
disp(table((0:4)', Ph, 100*Ph/sum(Ph), 'VariableNames', {'puiss','P_W','pct'}))