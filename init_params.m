% Paramètres d'un quadrirotor type (classe ~500g, bras 25cm)
params(1) = 0.5;        % m   [kg]      masse totale
params(2) = 3.0e-5;     % k_T [N/(rad/s)^2]
params(3) = 7.5e-7;     % k_Q [N.m/(rad/s)^2]  (k_Q/k_T ~ 0.025)
params(4) = 0.25;       % l   [m]       bras moteur
params(5) = 5.0e-3;     % Jxx [kg.m^2]
params(6) = 5.0e-3;     % Jyy [kg.m^2]
params(7) = 9.0e-3;     % Jzz [kg.m^2] (plus grand, axe vertical)
params(8) = 3.0e-5;     % Jr  [kg.m^2] inertie rotor