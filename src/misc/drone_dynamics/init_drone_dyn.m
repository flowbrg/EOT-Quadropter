clear; clc; close all;

% Parametres d'un quadrirotor type (classe ~500g, bras 25cm)
drone = struct();
drone.m   = 0.5;        % masse totale [kg]
drone.k_T = 3.0e-5;     % [N/(rad/s)^2]
drone.k_Q = 7.5e-7;     % [N.m/(rad/s)^2]  (k_Q/k_T ~ 0.025)
drone.l   = 0.25;       % bras moteur [m]
drone.Jxx = 5.0e-3;     % Moment d'inertie autour de xb [kg.m^2]
drone.Jyy = 5.0e-3;     % Moment d'inertie autour de yb [kg.m^2]
drone.Jzz = 9.0e-3;     % Moment d'inertie autour de zb [kg.m^2] (plus grand, axe vertical)
drone.w_h = sqrt(drone.m*9.81/(4*drone.k_T));

save("params.m","drone");