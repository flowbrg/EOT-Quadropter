% UAV
drone = struct();
drone.m   = 0.5;        % masse totale [kg]
drone.k_T = 3.0e-5;     % [N/(rad/s)^2]
drone.k_Q = 7.5e-7;     % [N.m/(rad/s)^2]  (k_Q/k_T ~ 0.025)
drone.l   = 0.25;       % bras moteur [m]
drone.Jxx = 5.0e-3;     % Moment d'inertie autour de xb [kg.m^2]
drone.Jyy = 5.0e-3;     % Moment d'inertie autour de yb [kg.m^2]
drone.Jzz = 9.0e-3;     % Moment d'inertie autour de zb [kg.m^2] (plus grand, axe vertical)
drone.w_h = sqrt(drone.m*9.81/(4*drone.k_T));  % equilibre [rad/s]

% Battery
batt = struct();
batt.C     = 1.3 * 3600;    % 1300 mAh = 4680 A.s
batt.Pol_ocv = [-55, 153, -155, 67.6, 0]; % V_oc(SoC) pol
batt.E_ocv = 11.1;          % 3S nominal [V]  (3 x 3.7V)
batt.R_int = 0.05;          % Resistance interne typique [Ohm]
batt.P_av  = 0;             % Avionique [W]

% BLDCM
mot.R     = 0.1;            % [Ohm]
mot.KE    = 0.01;           % [V.s/rad]
mot.Jr    = 3.0e-5;         % [kg.m^2]  (hypothese : = drone.Jr)
mot.Df    = 0; %1e-7;           % [N.m.s/rad]
mot.Tf    = 0;              % Frottement sec
mot.d     = drone.k_Q;      % d = k_Q
%mot.eta_m = 0.85;           % Rendement moteur brushless
mot.Kp = 9.4e-4;            % Coefficient propostionnel feedback pour Dc
mot.Ki = 9.42e-2;           % Coefficient integral feedback pour Dc


mot.w_h   = sqrt(drone.m*9.81/(4*drone.k_T));  % equilibre [rad/s]

mot.tau_m = mot.Jr / ((mot.KE^2/mot.R)+2*mot.d*mot.w_h); % Temps caractéristique du modèle simplifié d'ordre 1 du moteur

% Etat initial
xi0 = [0;0;0];
w0_motor = mot.w_h.*ones(4,1);
SoC0 = 1;

save("params.mat", "drone","batt", "mot", "xi0", "w0_motor", "SoC0");