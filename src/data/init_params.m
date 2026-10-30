% Parametres d'un quadrirotor type (classe ~500g, bras 25cm)
drone = struct();
drone.m   = 0.5;        % masse totale [kg]
drone.k_T = 3.0e-5;     % [N/(rad/s)^2]
drone.k_Q = 7.5e-7;     % [N.m/(rad/s)^2]  (k_Q/k_T ~ 0.025)
drone.l   = 0.25;       % bras moteur [m]
drone.Jxx = 5.0e-3;     % Moment d'inertie autour de xb [kg.m^2]
drone.Jyy = 5.0e-3;     % Moment d'inertie autour de yb [kg.m^2]
drone.Jzz = 9.0e-3;     % Moment d'inertie autour de zb [kg.m^2] (plus grand, axe vertical)
%drone.Jr  = 3.0e-5;     % inertie rotor [kg.m^2]

% params = [m; k_T; k_Q; l; Jxx; Jyy; Jzz;Jr];

% batt_params pour un LiPo 3S 1300mAh typique
battery = struct();
battery.C_batt  = 1.3 * 3600;    % 1300 mAh = 4680 A.s
battery.E_ocv   = 11.1;          % 3S nominal [V]  (3 x 3.7V)
battery.R0      = 0.05;          % Resistance interne typique [Ohm]
battery.eta_m   = 0.85;          % Rendement moteur brushless
battery.P_av    = 5.0;           % Avionique [W]

% batt_params = [C_batt; E_ocv; R0; eta_m];

% --- Paramètres moteur : PLACEHOLDERS a remplacer ---
mot.R     = 0.1;            % [Ohm]
mot.KE    = 0.01;           % [V.s/rad]
mot.Jm    = 3.0e-5;         % [kg.m^2]  (hypothese : = drone.Jr)
mot.Df    = 0; %1e-7;           % [N.m.s/rad]
mot.Tf    = 0;              % Frottement sec
mot.d     = drone.k_Q;      % d = k_Q

w_h      = sqrt(drone.m*9.81/(4*drone.k_T));  % equilibre [rad/s]

mot.tau_m = mot.Jm / ((mot.KE^2/mot.R)+2*mot.d*w_h); % Temps caractéristique du modèle simplifié d'ordre 1 du moteur

save("params.mat", "drone","battery", "mot");