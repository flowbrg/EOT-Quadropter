function dxdt = dyn_batt(states, commands, params)
% =========================================================
% Quadrotor augmenté avec modèle énergétique
% =========================================================
% ENTRÉES :
%   states      [17x1] : [x,y,z, vx,vy,vz, phi,theta,psi, p,q,r, w_i, SoC]
%   commands    [4x1]  : [w_r1,w_r2,w_r3,w_r4] en rad/s (>0)
%   params      [8x1]  : paramètres dynamiques (inchangés)
%   batt_params [4x1]  : [C_batt (A.s), E_ocv (V), R0 (Ohm), eta_m (-)]
%
% SORTIES :
%   dxdt        [13x1] : dérivées d'état

drone = params(1);
batt  = params(2);
mot   = params(3);

% Déballage batterie
C_batt  = batt.C_batt; % Capacité [A.s]
E_ocv   = batt.E_ocv;  % Tension OCV nominale [V]
R0      = batt.R0;     % Résistance interne [Ohm]
eta_m   = batt.eta_m;  % Rendement moteur
P_av    = batt.P_av;         % Puissance avionique [W] (constante)

omega      = states(13:16);  % vitesses rotors réelles [4x1]
omega_ref  = commands;       % consignes ESC [4x1]

% Dynamique moteur (1er ordre x4)
d_omega = (omega_ref - omega) / mot.tau_m;    % [4x1] rad/s2

% Dynamique 6-DOF (appel dyn)
dxdt_dyn = dyn(states(1:12), omega, drone);  % [12x1]

% Puissance (sur vitesses réelles, pas les consignes)
P_mec  = drone.k_Q .* omega.^3;     % k_Q*wi^3 [4x1]
P_elec = sum(P_mec) / eta_m + P_av; % [W]

% Courant & SoC
I    = P_elec / E_ocv; % [A]
dSoC = -I / C_batt;    % [s⁻¹]

% Vérification H5
if R0 * I / E_ocv > 0.05
    warning('Chute ohmique > 5%% (I = %.2f A)', I);
end

% Assemblage [17x1]
dxdt = [dxdt_dyn;   % 12
        d_omega;    %  4
        dSoC];      %  1
end