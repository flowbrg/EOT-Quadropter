function dxdt = dyn_energy(states, commands, params, batt_params)
% =========================================================
% Quadrotor augmenté avec modèle énergétique
% =========================================================
% ENTRÉES :
%   states      [13x1] : [x,y,z, vx,vy,vz, phi,theta,psi, p,q,r, SoC]
%   commands    [4x1]  : [w1,w2,w3,w4] en rad/s (>0)
%   params      [8x1]  : paramètres dynamiques (inchangés)
%   batt_params [4x1]  : [C_batt (A.s), E_ocv (V), R0 (Ohm), eta_m (-)]
%
% SORTIES :
%   dxdt        [13x1] : dérivées d'état

% --- Déballage batterie ---
C_batt  = batt_params(1);   % Capacité [A.s]
E_ocv   = batt_params(2);   % Tension OCV nominale [V]
R0      = batt_params(3);   % Résistance interne [Ohm]
eta_m   = batt_params(4);   % Rendement moteur
P_av    = 5.0;              % Puissance avionique [W] (constante)

% --- Dynamique 6-DOF (appel à dyn.m existant) ---
dxdt_dyn = dyn(states(1:12), commands, params);  % [12x1]

% --- Puissance moteur ---
w = commands;                           % [4x1] rad/s
P_mec  = params(3) .* w.^3;            % k_Q * wi^3 [W], [4x1]
P_elec = sum(P_mec) / eta_m + P_av;    % Scalaire [W]

% --- Courant batterie (modèle simplifié) ---
% Valide si R0*I << E_ocv
I = P_elec / E_ocv;                    % [A]

% --- Dynamique SoC ---
dSoC = -I / C_batt;                    % [s^-1], SoC ∈ [0,1]

% --- Vérification chute ohmique (assertion physique) ---
% Lever un warning si hypothèse H5 non vérifiée
if R0 * I / E_ocv > 0.05
    warning('Chute ohmique > 5%% : modèle simplifié moins précis (I=%.2f A)', I);
end

% --- Assemblage ---
dxdt = [dxdt_dyn; dSoC];   % [13x1]
end