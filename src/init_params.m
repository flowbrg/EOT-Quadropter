%% init_params.m

addpath("model/");

% UAV
drone.m   = 0.5;        % total mass [kg]
drone.k_T = 3.0e-5;     % [N/(rad/s)^2]
drone.k_Q = 7.5e-7;     % [N.m/(rad/s)^2]  (k_Q/k_T ~ 0.025)
drone.l   = 0.25;       % arm length [m]
drone.Jxx = 5.0e-3;     % Inertia around xb [kg.m^2]
drone.Jyy = 5.0e-3;     % Inertia around yb [kg.m^2]
drone.Jzz = 9.0e-3;     % Inertia around zb [kg.m^2]
drone.w_h = sqrt(drone.m*9.81/(4*drone.k_T));  % Hover rotor speed [rad/s]

% Battery
batt.C       = 1.3*3600;               % capacity: 1300 mAh = 4680 A.s
batt.Pol_ocv = [-55 153 -155 67.6 0];  % polynomial part of V_oc(SoC) (see dyn_uav)
batt.E_ocv   = 10.6; %11.1;                   % nominal open-circuit voltage, 3S LiPo [V]
batt.R_int   = 0.05;                   % internal resistance [Ohm]

% BLDCM
mot.R  = 0.1;            % winding resistance [Ohm]
mot.KE = 0.01;           % back-EMF constant [V.s/rad]
mot.Jr = 3.0e-5;         % rotor inertia [kg.m^2]
mot.d  = drone.k_Q;      % propeller load coefficient
mot.Kp = 9.4e-4;         % PI gains
mot.Ki = 9.42e-2;

% NLP Parameters
nlp.N  = 100;
nlp.nx = 16;  nlp.nu = 4;
nlp.P_ref = Pm_rows(drone.w_h*ones(4,1), zeros(4,1), drone, mot);   % [W], hover

% Initial states
xi0 = [0;0;0];                  % Initial position
w0_motor = drone.w_h.*ones(4,1);  % Initial rotor speeds
SoC0 = 1;                       % Initial state of charge
g0 = 9.81;                      % Acceleration due to gravity

save("data/params.mat", "drone","batt", "mot", "nlp", "xi0", "w0_motor", "SoC0", "g0");