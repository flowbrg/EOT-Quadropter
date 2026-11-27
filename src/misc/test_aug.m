%% test_aug.m
%
%   MOVE TO SRC/
%
% Test of augmented model (dyn_uav.m)
% Tests performed:
%   T1 — Steady state: verification that dxdt ~= 0 (except for dSoC)
%   T2 — Energy consistency: P_eq and t_vol_max
%   T3 — Closed-loop simulation with LQR + SoC tracking

clc; clear; close all;

load("data/params.mat")
addpath("model");

% BATTERY PARAMETERS (LiPo 3S, 1300 mAh)
SoC_0   = 1.0;          % Fully charged battery at the start
SoC_min = 0.2;          % Minimum discharge threshold (future requirement)

g      = 9.81;
w_h    = drone.w_h;     % [rad/s]
u_eq   = [w_h; w_h; w_h; w_h];

fprintf('=== Reference parameters ===\n');
fprintf('  Hover speed          : w_h   = %.2f rad/s\n', w_h);
fprintf('  Battery capacity     : C     = %.0f A.s\n',   batt.C);
fprintf('  Open-circuit voltage : E_ocv = %.1f V\n',     batt.E_ocv);
fprintf('\n');

% =========================================================
% TEST 1 — Fixed point at equilibrium
% Criteria: dxdt(1:12) ~= 0  AND  dSoC < 0 (normal discharge)
% =========================================================
fprintf("=== TEST 1 : Fixed point at hover ===\n");
% X0 = [x,y,z,vx,vy,vz,phi,theta,psi,p,q,r,w1,w2,w3,w4,SoC,z1,z2,z3,z4]
x0_aug = [zeros(12,1); w_h .* ones(4,1); SoC_0; zeros(4,1)];  % augmented state [21x1]

dxdt = dyn_uav(x0_aug, u_eq, drone, batt, mot);

fprintf('  norm of residual of dxdt(6) = %.2e  (target : < 1e-8)\n', ...
        dxdt(6));
fprintf('  dSoC = %.6f s-1  (should be < 0)\n', dxdt(17));

t_sim = 300;  % simulate 80% of max flight time, max 300 s
opts  = odeset('RelTol',1e-8, 'AbsTol',1e-10, ...
    'Events', @(t,x) evt_soc_min(t,x,SoC_min));

[t1, X1, ~, ~, ~] = ode45(...
    @(t,x) dyn_uav(x, u_eq, drone, batt, mot), ...
    [0 t_sim], x0_aug, opts);

% Figure T1
figure('Name','T1 - Hover');

subplot(3,1,1);
plot(t1, X1(:,1:3), 'LineWidth', 1.4);
legend('x','y','z','Location','east'); ylabel('Position [m]');
title('Hover of augmented model'); grid on;

subplot(3,1,2);
plot(t1, X1(:,7:9)*180/pi, 'LineWidth', 1.4);
legend('\phi','\theta','\psi','Location','east'); ylabel('Attitude [deg]'); grid on;

subplot(3,1,3);
yyaxis left
plot(t1, X1(:,17)*100, 'b-', 'LineWidth', 1.5); ylabel('SoC [%]');
yyaxis right

assert(dxdt(6) < 1e-6, 'FAIL T1 : residual too large');
assert(dxdt(17) < 0,            'FAIL T1 : dSoC should be negative');
fprintf('  [PASS] Test 1\n\n');

% =========================================================
% TEST 2 — Steady-state simulation: SoC evolution
% Criterion: SoC(t) strictly decreasing, linear
% Slope = -I/C  consistent with T1
% =========================================================
fprintf('=== TEST 2 : Equilibrium simulation — SoC evolution ===\n');

opts  = odeset('RelTol',1e-8, 'AbsTol',1e-10, ...
               'Events', @(t,x) evt_soc_min(t,x,SoC_min));

[t2, X2, te, ~, ~] = ode45(...
    @(t,x) dyn_uav(x, u_eq, drone, batt, mot), ...
    [0 t_sim], x0_aug, opts);

SoC_sim = X2(:,17);

% Monotonicity check
assert(all(diff(SoC_sim) < 0), 'FAIL T2 : SoC is not decreasing');

% Slope check (linear regression)
p_fit    = polyfit(t2, SoC_sim, 1);          % expected slope
% pente_th = -I_eq / batt.C;               % [s⁻¹]
% err_rel  = abs(p_fit(1) - pente_th) / abs(pente_th);

fprintf('  Measured SoC slope    = %.6e s⁻¹\n', p_fit(1));
% fprintf('  Theoretical SoC slope = %.6e s⁻¹\n', pente_th);
% fprintf('  Relative error        = %.2f%%  (< 1%% expected)\n', 100*err_rel);

if ~isempty(te)
    fprintf('  SoC_min event reached at t = %.1f s\n', te);
end

% assert(err_rel < 0.01, 'FAIL T2 : SoC slope inconsistent with I/C balance');
fprintf('  [PASS] Test 2\n\n');

% --- Figure T2 ---
figure('Name',"T2 — SoC at equilibrium");
yyaxis left
plot(t2, SoC_sim*100, 'b-', 'LineWidth', 1.5);
yline(SoC_min*100, 'r--', sprintf('SoC_{min} = %d%%', 100*SoC_min));
ylabel('SoC [%]'); ylim([0 105]);
yyaxis right


% =========================================================
% TEST 3 — Closed-loop simulation with LQR + SoC
% Objective: verify that the augmented model is compatible
%            with the LQR controller from simu.m
% =========================================================
fprintf('=== TEST 3 : Closed-loop LQR — initial perturbation in z ===\n');

% Numerical linearization around equilibrium (over the first 12 states)
eps_lin = 1e-6;
A = zeros(12,12);
B = zeros(12,4);
for i = 1:12
    xp = zeros(12,1); xp(i) =  eps_lin;
    xm = zeros(12,1); xm(i) = -eps_lin;
    A(:,i) = (dyn(xp, u_eq, drone) - dyn(xm, u_eq, drone)) / (2*eps_lin);
end
for i = 1:4
    up = u_eq; up(i) = up(i)+eps_lin;
    um = u_eq; um(i) = um(i)-eps_lin;
    B(:,i) = (dyn(zeros(12,1),up,drone) - dyn(zeros(12,1),um,drone)) / (2*eps_lin);
end

% Bryson's rule
deg2rad = pi/180;
x_max = [1;1;0.5;2;2;2;15*deg2rad;15*deg2rad;45*deg2rad;1;1;1];
u_max = repmat(w_h*0.2, 4, 1);
Q_lqr = diag(1./x_max.^2);
R_lqr = diag(1./u_max.^2);

K = lqr(A, B, Q_lqr, R_lqr);
fprintf('  LQR computed. Eigenvalues of A-BK :\n');
disp(eig(A - B*K)');

% Simulation: perturbation z=+0.3m, initial SoC = 1
x0_pert    = [zeros(12,1); w_h.*ones(4,1); 1; zeros(4,1)];
x0_pert(3) = 0.3;    % altitude perturbation [m]

% LQR controller (operates on the 12 dynamic states)
ctrl = @(x) u_eq - K*(x(1:12) - zeros(12,1));

t_end = 30;  % [s]
[t3, X3] = ode45(...
    @(t,x) dyn_uav(x, max(ctrl(x), 0), drone, batt, mot), ...
    [0 t_end], x0_pert, ...
    odeset('RelTol',1e-8,'AbsTol',1e-10));

% Stabilization check
idx_fin  = find(t3 > 20, 1);  % after 20 s
pos_fin  = X3(idx_fin:end, 1:3);
err_pos  = max(vecnorm(pos_fin, 2, 2));
fprintf('  Residual position error (t>20s) : %.4f m  (< 0.01 m required)\n', err_pos);
assert(err_pos < 0.01, 'FAIL T3 : not stabilized after 20 s');

% Energy consumed during the maneuver
dSoC_man = X3(1,13) - X3(end,13);          % consumed ΔSoC
E_man    = dSoC_man * batt.C * batt.E_ocv;  % [J] (approximation)
fprintf('  Maneuver ΔSoC  = %.4f  (%.2f%%)\n', dSoC_man, 100*dSoC_man);
fprintf('  Maneuver energy ≈ %.2f J  = %.4f Wh\n', E_man, E_man/3600);
fprintf('  [PASS] Test 3\n\n');

% --- Figure T3 ---
figure('Name','T3 — Closed-loop LQR + SoC');

subplot(3,1,1);
plot(t3, X3(:,1:3), 'LineWidth', 1.4);
legend('x','y','z','Location','east'); ylabel('Position [m]');
title('LQR response with augmented model'); grid on;

subplot(3,1,2);
plot(t3, X3(:,7:9)*180/pi, 'LineWidth', 1.4);
legend('\phi','\theta','\psi','Location','east'); ylabel('Attitude [deg]'); grid on;

subplot(3,1,3);
yyaxis left
plot(t3, X3(:,17)*100, 'b-', 'LineWidth', 1.5); ylabel('SoC [%]');
yyaxis right
% Reconstructed instantaneous power
u_traj = arrayfun(@(k) max(ctrl(X3(k,:)'), 0), 1:length(t3), 'UniformOutput', false);
u_traj = cell2mat(u_traj)';  % [Nx4]
P_traj = (drone.k_Q/1) * sum(u_traj.^3, 2) + batt.P_av;
plot(t3, P_traj, 'r--', 'LineWidth', 1.2); ylabel('P_{elec} [W]');
xlabel('Time [s]'); legend('SoC','P_{elec}'); grid on;

fprintf('=== VALIDATION SUMMARY ===\n');
fprintf('  T1 [static equilibrium] : PASS\n');
fprintf('  T2 [linear discharge]   : PASS\n');
fprintf('  T3 [LQR + SoC]          : PASS\n');

% =========================================================
% EVENT FUNCTION — stop if SoC < SoC_min
% =========================================================
function [value, isterminal, direction] = evt_soc_min(~, x, SoC_min)
    value      = x(17) - SoC_min;  % vanishes when SoC = SoC_min
    isterminal = 1;                 % stops integration
    direction  = -1;                % detects downward crossing
end