function dXdt = dyn_uav(X, U, drone, batt, mot)
% =========================================================
% Complete UAV dynamic model
% =========================================================
% States :  [21x4]
%           (1:3)   [x, y, z]
%           (4:6)   [vx, vy, vz]
%           (7:9)   [phi, theta, psi]
%           (10:12) [p, q, r]
%           (13:16) [w_1, w_2, w_3, w_4]
%           (17)    SoC
%           (18:21) [z_1, z_2, z_3, z_4]


% Duty-cycle
[Dc, dz] = dutycycle(U, X(13:16), X(18:21), batt, mot);

% Battery Open Circuit Voltage
V_oc = polyval(batt.Pol_ocv,X(17)) + log(X(17))*X(17);

% BLDCM dynamics
[I_batt, dw] = dyn_bldcm(Dc,V_oc,X(13:16),batt,mot);

% Battery state of charge
dSoC = -I_batt/batt.C;
% Debug
% fprintf('SoC = %.4f | I_batt = %.4f A | dSoC = %.6e\n', ...
%    X(17), I_batt, dSoC);
% fprintf('U1 = %.4f | w1 = %.4f\n', ...
%    U(1), X(13));

% Drone dynamics
%dxdt_dyn = dyn(X(1:12), U, drone);  % [12x1]
dxdt_dyn = dyn(X(1:12), X(13:16), drone);  % [12x1]


% Assemblage [17x1]
dXdt = [dxdt_dyn;   % 12
        dw;         % 4
        dSoC;       % 1
        dz;];       % 4
end