% Parametres d'un quadrirotor type (classe ~500g, bras 25cm)
m   = 0.5;        % masse totale [kg]
k_T = 3.0e-5;     % [N/(rad/s)^2]
k_Q = 7.5e-7;     % [N.m/(rad/s)^2]  (k_Q/k_T ~ 0.025)
l   = 0.25;       % bras moteur [m]
Jxx = 5.0e-3;     % Moment d'inertie autour de xb [kg.m^2]
Jyy = 5.0e-3;     % Moment d'inertie autour de yb [kg.m^2]
Jzz = 9.0e-3;     % Moment d'inertie autour de zb [kg.m^2] (plus grand, axe vertical)
Jr  = 3.0e-5;     % inertie rotor [kg.m^2]

params = [m; k_T; k_Q; l; Jxx; Jyy; Jzz;Jr];

% batt_params pour un LiPo 3S 1300mAh typique
C_batt  = 1.3 * 3600;    % 1300 mAh → 4680 A.s
E_ocv   = 11.1;          % 3S nominal [V]  (3 x 3.7V)
R0      = 0.05;          % Resistance interne typique [Ohm]
eta_m   = 0.85;          % Rendement moteur brushless
P_av    = 5.0;           % Avionique [W]

batt_params = [C_batt; E_ocv; R0; eta_m];