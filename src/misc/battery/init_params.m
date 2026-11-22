% Battery
batt.C     = 1.3 * 3600;    % 1300 mAh = 4680 A.s
batt.Pol_ocv = [-55, 153, -155, 67.6, 0]; % V_oc(SoC) pol
batt.E_ocv = 11.1;          % 3S nominal [V]  (3 x 3.7V)
batt.R_int = 0.05;          % Resistance interne typique [Ohm]