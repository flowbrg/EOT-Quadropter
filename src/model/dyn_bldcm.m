function [I_batt, dw] = dyn_bldcm(Dc, V_oc, w, batt, mot)

% Terminal voltage
A = sum(Dc.^2);
B = mot.KE*sum(Dc.*w);

% Terminal Voltage
V_term  = (mot.R*V_oc+batt.R_int*B)/(mot.R+batt.R_int*A);

% Debug
%fprintf('V_term = %.4f V | V_oc = %.4f V |\n', V_term, V_oc);


% Electrical Equation
% Note: L~0 => L*di=0
i_bldcm = (V_term.*Dc - mot.KE.*w)/mot.R;

% Battery current
%I_batt = (V_oc - V_term)/batt.R_int;
I_batt = sum(Dc.*i_bldcm);

% Mechanical equation
dw = (mot.KE.*i_bldcm - mot.d*w.^2)/mot.Jr;
end