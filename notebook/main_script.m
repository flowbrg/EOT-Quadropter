%% Energy-optimal trajectory planning for a quadrotor (fmincon, interior-point, exact Hessian)
%  1. Parameters   2. NLP definition   3. Solve   4. Simulation (21-state model)   5. Plots
%  Required: Optimization Toolbox, Symbolic Math Toolbox (Hessian block, regenerated at each run).
%  All functions are local functions at the end of this file.

clear; clc; close all;

%% 1. Parameters
% --- UAV
drone.m   = 0.5;         % mass [kg]
drone.g   = 9.81;        % gravity [m/s^2]
drone.k_T = 3.0e-5;      % thrust coefficient [N/(rad/s)^2]
drone.k_Q = 7.5e-7;      % reaction-torque coefficient [N.m/(rad/s)^2]
drone.l   = 0.25;        % arm length [m]
drone.Jxx = 5.0e-3;      % inertia [kg.m^2]
drone.Jyy = 5.0e-3;
drone.Jzz = 9.0e-3;
drone.w_h = sqrt(drone.m*drone.g/(4*drone.k_T));   % hover rotor speed [rad/s]

% --- Battery
batt.C       = 1.3*3600;               % capacity: 1300 mAh = 4680 A.s
batt.Pol_ocv = [-55 153 -155 67.6 0];  % polynomial part of V_oc(SoC) (see dyn_uav)
batt.E_ocv   = 11.1;                   % nominal open-circuit voltage, 3S LiPo [V]
batt.R_int   = 0.05;                   % internal resistance [Ohm]

% --- BLDC motor and duty-cycle controller
mot.R  = 0.1;            % winding resistance [Ohm]
mot.KE = 0.01;           % back-EMF constant [V.s/rad]
mot.Jr = 3.0e-5;         % rotor inertia [kg.m^2]
mot.d  = drone.k_Q;      % propeller load coefficient
mot.Kp = 9.4e-4;         % PI gains
mot.Ki = 9.42e-2;

% --- Scene
scen.start_pos  = [0 0 0];     % [m]
scen.target_pos = [50 50 0];   % [m]
scen.start_vel  = [0 0 0];     % [m/s]
scen.target_vel = [0 0 0];     % [m/s] soft landing

% Buildings, one row each: [xc yc dx dy hz]
%   (xc,yc): centre at ground level; dx, dy: half horizontal sizes; hz: height -> z in [0, hz]
scen.buildings = [
    10, 10, 3, 4, 12;   % B1 thin and high
    30, 15, 5, 3,  7;   % B2 large
    27, 40, 4, 4, 15;   % B3 square, high
    35, 25, 3, 6,  9;   % B4 thin along y
    10, 25, 5, 5, 11;   % B5 square, high
    25, 25, 2, 2,  5];  % B6 in the middle, small
scen.n_buildings = size(scen.buildings, 1);
scen.ecart = 1.0;                                  % safety distance to buildings [m]

scen.x_min = 0;  scen.x_max = 55;                  % flight volume [m]
scen.y_min = 0;  scen.y_max = 55;
scen.z_min = 0;  scen.z_max = 30;

draw_scene(scen); legend('Location','northwest');

%% 2. NLP definition
% State  X = [x y z | vx vy vz | phi theta psi | p q r | w1 w2 w3 w4]  (16)
% Input  U = [dw1 dw2 dw3 dw4]  rotor accelerations [rad/s^2]            (4)
% Battery SoC and PI states are not in the NLP; they only appear in the simulation (section 4).
Tf_init = 20;  Tf_min = 8;  Tf_max = 40;   % final time [s]
N       = 60;                              % collocation intervals
nx = 16;  nu = 4;
w_h     = drone.w_h;
dw_max  = 500;                             % rotor acceleration bound [rad/s^2]
ang_lim = deg2rad(35);                     % bound on phi and theta [rad]
c_T     = 1;                               % weight of the final time in the cost [-]
p_sq    = 4;                               % super-ellipsoid exponent (even; larger = boxier but worse conditioning)

p0 = scen.start_pos(:);   pf = scen.target_pos(:);
bld_c = scen.buildings(:,1:2);                 % centres (xc, yc)
bld_a = scen.buildings(:,3:5) + scen.ecart;    % semi-axes with safety margin (ax, ay, az)
P_ref = Pm_rows(w_h*ones(4,1), zeros(4,1), drone, mot);   % hover power [W], normalises the cost

assert(norm(dyn_vec([zeros(12,1); w_h*ones(4,1)], zeros(4,1), drone)) < 1e-9, 'Hover is not an equilibrium');

% --- Decision variables (with bounds)
x_lb = [scen.x_min; scen.y_min; scen.z_min; -inf(3,1); -ang_lim; -ang_lim; -inf; -inf(3,1); 0.1*w_h*ones(4,1)];
x_ub = [scen.x_max; scen.y_max; scen.z_max;  inf(3,1);  ang_lim;  ang_lim;  inf;  inf(3,1); 1.8*w_h*ones(4,1)];

Tf = optimvar('Tf', 1,       'LowerBound', Tf_min,             'UpperBound', Tf_max);
X  = optimvar('X',  nx, N+1, 'LowerBound', repmat(x_lb,1,N+1), 'UpperBound', repmat(x_ub,1,N+1));
U  = optimvar('U',  nu, N+1, 'LowerBound', -dw_max,            'UpperBound', dw_max);

% --- Cost: electrical energy (Simpson) normalised by P_ref, plus final-time penalty
h    = Tf/N;                                         % collocation step
prob = optimproblem('Objective', elec_energy(X(13:16,:), U, h, drone, mot)/P_ref + c_T*Tf);

% --- Constraints
prob.Constraints.colloc = (colloc_defects(X, U, h, drone) == 0);   % trapezoidal collocation

% Boundary conditions (final yaw free)
prob.Constraints.init   = (X(:,1)       == [p0; scen.start_vel(:); zeros(6,1); w_h*ones(4,1)]);
prob.Constraints.pos_f  = (X(1:3,N+1)   == pf);
prob.Constraints.vel_f  = (X(4:6,N+1)   == scen.target_vel(:));
prob.Constraints.att_f  = (X(7:8,N+1)   == 0);
prob.Constraints.rate_f = (X(10:12,N+1) == 0);
prob.Constraints.rot_f  = (X(13:16,N+1) == w_h*ones(4,1));

% Obstacles: stay outside each super-ellipsoid at every node (one vector constraint per building)
for b = 1:scen.n_buildings
    prob.Constraints.(sprintf('obst_%d', b)) = ...
        (superellipsoid(X(1:3,:), bld_c(b,:), bld_a(b,:), p_sq) >= 1);
end

% --- Initial guess: smoothstep in (x,y) + altitude plateau above the tallest building
tau   = linspace(0, 1, N+1);
sprof = 3*tau.^2 - 2*tau.^3;
dsdt  = (6*tau - 6*tau.^2) / Tf_init;

z_cruise = max(scen.buildings(:,5)) + 2*scen.ecart;
assert(z_cruise < scen.z_max, 'z_cruise = %.1f m must be below z_max = %.1f m', z_cruise, scen.z_max);
sstep  = @(a) 3*min(max(a,0),1).^2 - 2*min(max(a,0),1).^3;
z_prof = z_cruise*(sstep(tau/0.15) - sstep((tau-0.85)/0.15));

X_init = zeros(nx, N+1);
X_init(1:3,:)   = p0 + (pf-p0)*sprof;
X_init(3,:)     = X_init(3,:) + z_prof;
X_init(4:6,:)   = (pf-p0)*dsdt;
X_init(6,:)     = X_init(6,:) + gradient(z_prof, Tf_init/N);
X_init(13:16,:) = w_h;

x0 = struct('Tf', Tf_init, 'X', X_init, 'U', zeros(nu,N+1));

%% 3. Solve
% Gradient and Jacobians: automatic differentiation (prob2struct).
% Hessian of the Lagrangian: exact, from the symbolic block of ONE collocation interval
% (hess_loc.m, rewritten here at each run), assembled sparsely in hess_lagr.
solverProb = prob2struct(prob, x0, ...
    'ObjectiveDerivative',   'auto-reverse', ...
    'ConstraintDerivative',  'auto-reverse', ...
    'ObjectiveFunctionName', 'fmincon_objective', ...
    'ConstraintFunctionName','fmincon_constraints');

idx = varindex(prob);                                  % position of Tf, X, U in the solver vector z
gen_hess_loc(drone, mot, N, P_ref);
Hfun = make_hess_fcn(idx, bld_c, bld_a, p_sq, N);

solverProb.options = optimoptions('fmincon', ...
    'Algorithm',                'interior-point', ...
    'HessianFcn',               Hfun, ...
    'SpecifyObjectiveGradient', true, ...
    'SpecifyConstraintGradient',true, ...
    'ScaleProblem',             true, ...
    'MaxFunctionEvaluations',   1e7, ...
    'MaxIterations',            2000, ...
    'OptimalityTolerance',      1e-5, ...
    'ConstraintTolerance',      1e-5, ...
    'Display',                  'iter');              % 'iter' to follow the convergence

tic
[z_sol, ~, exitflag, output] = fmincon(solverProb);
t_solve = toc;

Tf_sol = z_sol(idx.Tf);
X_sol  = reshape(z_sol(idx.X), nx, N+1);
U_sol  = reshape(z_sol(idx.U), nu, N+1);               % rotor accelerations; rotor speeds are X_sol(13:16,:)
t_sol  = linspace(0, Tf_sol, N+1);
E_sol  = elec_energy(X_sol(13:16,:), U_sol, Tf_sol/N, drone, mot);

for b = 1:scen.n_buildings                             % a posteriori check: min s_b must be >= 1
    fprintf('Building %d: min s_b = %.4f (>= 1 required)\n', b, ...
        min(superellipsoid(X_sol(1:3,:), bld_c(b,:), bld_a(b,:), p_sq)));
end
fprintf('Tf = %.2f s | E = %.1f J (hover over Tf: %.1f J)\n', Tf_sol, E_sol, P_ref*Tf_sol);
fprintf('exitflag = %d | %d iterations | constraint violation = %.2e | %.1f s (%.3f s/iter)\n', ...
    exitflag, output.iterations, output.constrviolation, t_solve, t_solve/max(output.iterations,1));

%% 4. Simulation on the 21-state model (open loop, rotor-speed reference held constant on each interval)
% State: [12 rigid-body | 4 rotor speeds | SoC | 4 PI integrators]
W_ref = X_sol(13:16,:);
SoC0  = 1.0;
x_c   = [X_sol(:,1); SoC0; zeros(4,1)];
odeopts = odeset('RelTol',1e-5, 'AbsTol',1e-7);

t_real = t_sol(1);
x_real = x_c.';
for k = 1:N
    [t_k, x_k] = ode45(@(t,x) dyn_uav(x, W_ref(:,k), drone, batt, mot), t_sol(k:k+1), x_c, odeopts);
    t_real = [t_real; t_k(2:end)];
    x_real = [x_real; x_k(2:end,:)];
    x_c    = x_k(end,:).';
end

x_end = x_real(end,:);
fprintf('Final errors | position: %.4f m | speed: %.4f m/s | attitude: %.4f rad | final SoC: %.5f\n', ...
    norm(x_end(1:3).' - pf), norm(x_end(4:6).' - scen.target_vel(:)), norm(x_end(7:8)), x_end(17));

%% 5. Plots
% --- 3D view
draw_scene(scen);
plot3(X_sol(1,:),  X_sol(2,:),  X_sol(3,:),  'b-', 'LineWidth',2, 'DisplayName','Ideal (fmincon)');
plot3(x_real(:,1), x_real(:,2), x_real(:,3), 'r-', 'LineWidth',2, 'DisplayName','Simulated');
legend('Location','northwest');

% --- Top view with horizontal-velocity arrows
figure; hold on; grid on; axis equal; xlabel('x (m)'); ylabel('y (m)');
xlim([scen.x_min-1 scen.x_max+1]);  ylim([scen.y_min-1 scen.y_max+1]);
for b = 1:scen.n_buildings
    bd = scen.buildings(b,:);
    rectangle('Position', [bd(1)-bd(3), bd(2)-bd(4), 2*bd(3), 2*bd(4)], ...
        'FaceColor',[0.8 0.2 0.2 0.5], 'EdgeColor','k', 'HandleVisibility','off');
    text(bd(1), bd(2), sprintf('B%d (%.0f m)', b, bd(5)), ...
        'HorizontalAlignment','center', 'FontSize',8, 'HandleVisibility','off');
end
plot(p0(1), p0(2), 'gs', 'MarkerSize',12, 'MarkerFaceColor','g', 'DisplayName','Start');
plot(pf(1), pf(2), 'p',  'MarkerSize',16, 'MarkerFaceColor','y', 'MarkerEdgeColor','k', 'DisplayName','Target');
plot(X_sol(1,:),  X_sol(2,:),  'b-', 'LineWidth',2, 'DisplayName','Ideal trajectory');
plot(x_real(:,1), x_real(:,2), 'r-', 'LineWidth',2, 'DisplayName','Actual trajectory');
plot_heading(X_sol(1,:),  X_sol(2,:),  X_sol(4,:),  X_sol(5,:),  'k');
plot_heading(x_real(:,1), x_real(:,2), x_real(:,4), x_real(:,5), [0.85 0.325 0.098]);
legend('Location','best');

% --- States: ideal (solid) vs simulated (dashed)
groups = { 1:3,   1,      'Position (m)',         {'x','y','z'};
           4:6,   1,      'Speed (m/s)',          {'v_x','v_y','v_z'};
           7:9,   180/pi, 'Attitude (deg)',       {'\phi','\theta','\psi'};
           10:12, 1,      'Angular rate (rad/s)', {'p','q','r'} };
col = lines(3);
for g = 1:size(groups,1)
    [rows, scale, lbl, names] = groups{g,:};
    figure; hold on; grid on;
    for i = 1:3
        plot(t_sol,  scale*X_sol(rows(i),:),  '-',  'Color',col(i,:), 'LineWidth',1.5, 'DisplayName',['ideal '  names{i}]);
        plot(t_real, scale*x_real(:,rows(i)), '--', 'Color',col(i,:), 'LineWidth',1.5, 'DisplayName',['actual ' names{i}]);
    end
    xlabel('Time (s)'); ylabel(lbl); title(lbl); legend('Location','best','NumColumns',3);
end

% --- Battery state of charge
figure; hold on; grid on;
plot(t_real, x_real(:,17), 'k-', 'LineWidth',1.8);
yline(SoC0, 'b:', 'Initial SoC');
xlabel('Time (s)'); ylabel('SoC (-)'); title('Battery state of charge');

% --- Rotor speeds: reference vs actual
figure;
for i = 1:4
    subplot(2,2,i); hold on; grid on;
    plot(t_sol,  W_ref(i,:),     'b-', 'LineWidth',1.8, 'DisplayName','Reference');
    plot(t_real, x_real(:,12+i), 'r-', 'LineWidth',1.5, 'DisplayName','Actual');
    yline(w_h, 'g:', 'HandleVisibility','off');
    xlabel('Time (s)'); ylabel(sprintf('\\omega_%d (rad/s)', i)); title(sprintf('Motor %d', i));
    if i == 1, legend('Location','best'); end
end

% --- Optimal commands (rotor accelerations)
figure;
for i = 1:4
    subplot(2,2,i); grid on;
    plot(t_sol, U_sol(i,:), 'b-', 'LineWidth',1.8);
    xlabel('Time (s)'); ylabel(sprintf('d\\omega_{%d}/dt (rad/s^2)', i)); title(sprintf('Rotor %d command', i));
end

%% ============================== Local functions ==============================

% ---------- Dynamics ----------
function dX = dyn(X, w, drone)
% Rigid-body dynamics (12 states), rotor speeds w (4 x M) as input.
% Element-wise: works for a single column (simulation) and for M nodes at once (NLP,
% numeric / symbolic / optimization expressions).
    vx = X(4,:);   vy = X(5,:);     vz = X(6,:);
    phi = X(7,:);  th = X(8,:);     psi = X(9,:);
    p = X(10,:);   q = X(11,:);     r = X(12,:);

    cphi = cos(phi);  sphi = sin(phi);
    cth  = cos(th);   sth  = sin(th);   tth = sth./cth;
    cpsi = cos(psi);  spsi = sin(psi);

    w2   = w.^2;
    T    = drone.k_T*(w2(1,:) + w2(2,:) + w2(3,:) + w2(4,:));      % total thrust
    Gphi = drone.l*drone.k_T*(w2(4,:) - w2(2,:));                  % torques
    Gth  = drone.l*drone.k_T*(w2(3,:) - w2(1,:));
    Gpsi = drone.k_Q*(w2(1,:) - w2(2,:) + w2(3,:) - w2(4,:));

    % Translation: (R*[0;0;T] - [0;0;m g])/m, third column of R
    ax = (cpsi.*sth.*cphi + spsi.*sphi).*T/drone.m;
    ay = (spsi.*sth.*cphi - cpsi.*sphi).*T/drone.m;
    az = (cth.*cphi).*T/drone.m - drone.g;

    % Euler-angle kinematics: deta = W(eta)*[p;q;r]
    dphi = p + sphi.*tth.*q + cphi.*tth.*r;
    dth  = cphi.*q - sphi.*r;
    dpsi = (sphi.*q + cphi.*r)./cth;

    % Euler's equation (diagonal inertia): J*dom = -om x (J om) + G
    dp = (-(drone.Jzz - drone.Jyy).*q.*r + Gphi)/drone.Jxx;
    dq = (-(drone.Jxx - drone.Jzz).*r.*p + Gth )/drone.Jyy;
    dr = (-(drone.Jyy - drone.Jxx).*p.*q + Gpsi)/drone.Jzz;

    dX = [vx; vy; vz; ax; ay; az; dphi; dth; dpsi; dp; dq; dr];
end

function F = dyn_vec(X, U, drone)
% NLP dynamics (16 states): rotor speeds are states, rotor accelerations U are the inputs.
    F = [dyn(X(1:12,:), X(13:16,:), drone); U];
end

function [Dc, dz] = dutycycle(w_ref, w, z, batt, mot)
% Feedforward (steady-state motor equations) + PI feedback. Inputs 4x1.
% Dc: duty cycle in [0,1];  dz: derivative of the PI integrator state.
    e   = w_ref - w;
    Dff = (mot.R*mot.d)/(mot.KE*batt.E_ocv)*w_ref.^2 + mot.KE/batt.E_ocv*w_ref;
    Dc  = min(1, max(Dff + mot.Kp*e + mot.Ki*z, 0));
    dz  = e;
end

function [I_batt, dw] = dyn_bldcm(Dc, V_oc, w, batt, mot)
% BLDC motors + battery. The algebraic loop (V_term <-> motor currents) is solved in closed form;
% quasi-static current (L di/dt = 0). Dc, w: 4x1.
    A = sum(Dc.^2);
    B = mot.KE*sum(Dc.*w);
    V_term = (mot.R*V_oc + batt.R_int*B)/(mot.R + batt.R_int*A);   % battery terminal voltage
    i_m    = (V_term*Dc - mot.KE*w)/mot.R;                         % motor currents
    I_batt = sum(Dc.*i_m);                                         % battery current
    dw     = (mot.KE*i_m - mot.d*w.^2)/mot.Jr;                     % rotor accelerations
end

function dX = dyn_uav(X, w_ref, drone, batt, mot)
% Complete model (21 states), input: reference rotor speeds w_ref (4x1).
%   X = [x y z vx vy vz phi theta psi p q r (1:12) | w1..w4 (13:16) | SoC (17) | z1..z4 PI (18:21)]
    [Dc, dz]     = dutycycle(w_ref, X(13:16), X(18:21), batt, mot);
    V_oc         = polyval(batt.Pol_ocv, X(17)) + log(X(17))*X(17);
    [I_batt, dw] = dyn_bldcm(Dc, V_oc, X(13:16), batt, mot);
    dX = [dyn(X(1:12), X(13:16), drone); dw; -I_batt/batt.C; dz];
end

% ---------- NLP building blocks (shared by the problem and by the Hessian block) ----------
function P = Pm_rows(W, dW, drone, mot)
% Electrical motor power [W] at each node, summed over the 4 rotors. W, dW: 4 x M -> P: 1 x M.
    a  = mot.Jr*dW + drone.k_Q*W.^2;
    Pr = (mot.R/mot.KE^2)*a.^2 + W.*a;
    P  = Pr(1,:) + Pr(2,:) + Pr(3,:) + Pr(4,:);
end

function J = elec_energy(W, dW, h, drone, mot)
% Electrical energy [J]: Simpson rule on each interval of length h. W, dW: 4 x (n+1) (n intervals).
    n  = size(W,2) - 1;
    Wa = W(:,1:n);    Wb = W(:,2:n+1);    dWa = dW(:,1:n);    dWb = dW(:,2:n+1);
    P  = Pm_rows(Wa, dWa, drone, mot) + 4*Pm_rows((Wa+Wb)/2, (dWa+dWb)/2, drone, mot) + Pm_rows(Wb, dWb, drone, mot);
    J  = h/6*sum(P);
end

function d = colloc_defects(X, U, h, drone)
% Trapezoidal collocation defects X_{k+1} - X_k - h/2 (f_k + f_{k+1}), 16 x n.
    n = size(X,2) - 1;
    F = dyn_vec(X, U, drone);
    d = X(:,2:n+1) - X(:,1:n) - h/2*(F(:,1:n) + F(:,2:n+1));
end

function s = superellipsoid(P, c, a, p)
% s >= 1 <=> point (columns of P, 3 x M) outside the super-ellipsoid centred at (c(1), c(2), 0)
% with semi-axes a(1:3) and exponent p.
    s = ((P(1,:) - c(1))/a(1)).^p + ((P(2,:) - c(2))/a(2)).^p + (P(3,:)/a(3)).^p;
end

% ---------- Exact Hessian of the Lagrangian  L = f + lam_ineq'*c + lam_eq'*ceq ----------
function gen_hess_loc(drone, mot, N, P_ref)
% Writes hess_loc.m: Hessian (41 x 41) of the Lagrangian of ONE collocation interval,
% w.r.t. zl = [Tf; x_k; u_k; x_{k+1}; u_{k+1}], for multipliers lam (16 x 1) of its defects.
% (The term c_T*Tf is linear: no contribution.)
    syms Tf_s real
    xa  = sym('xa',  [16 1], 'real');   ua = sym('ua', [4 1], 'real');
    xb  = sym('xb',  [16 1], 'real');   ub = sym('ub', [4 1], 'real');
    lam = sym('lam', [16 1], 'real');
    zl  = [Tf_s; xa; ua; xb; ub];

    X = [xa xb];   U = [ua ub];   h = Tf_s/N;
    L = elec_energy(X(13:16,:), U, h, drone, mot)/P_ref + lam.'*colloc_defects(X, U, h, drone);

    matlabFunction(hessian(L, zl), 'File','hess_loc', 'Vars',{zl, lam}, 'Optimize',true);
    rehash;  clear hess_loc
end

function Hfun = make_hess_fcn(idx, c, a, p, N)
% Pre-computes the sparse assembly pattern and returns the HessianFcn handle for fmincon.
    nx = numel(idx.X)/(N+1);   nu = numel(idx.U)/(N+1);
    IX = reshape(idx.X, nx, N+1);
    IU = reshape(idx.U, nu, N+1);
    LOC = [repmat(idx.Tf,1,N); IX(:,1:N); IU(:,1:N); IX(:,2:N+1); IU(:,2:N+1)];   % 41 x N, rows = zl of hess_loc
    nl  = size(LOC,1);

    hs.N   = N;
    hs.nz  = numel(idx.Tf) + numel(idx.X) + numel(idx.U);
    hs.LOC = LOC;
    hs.I   = reshape(repmat(reshape(LOC, nl, 1, N), [1 nl 1]), [], 1);   % (row, col) of each local block entry,
    hs.J   = reshape(repmat(reshape(LOC, 1, nl, N), [nl 1 1]), [], 1);   % column-major order of Hk(:)
    hs.IP  = IX(1:3,:);                                                  % indices of (x,y,z) at each node
    hs.c = c;   hs.a = a;   hs.p = p;   hs.nb = size(c,1);

    Hfun = @(z, lambda) hess_lagr(z, lambda, hs);
end

function H = hess_lagr(z, lambda, hs)
% Contributions:
%   1) cost + collocation: sum of the local 41x41 blocks (sparse() adds the overlaps of neighbouring intervals);
%   2) obstacles c_b = 1 - s_b: diagonal, d2c/dr_i^2 = -p(p-1)(r_i - c_i)^(p-2)/a_i^p;
%   3) bounds, boundary conditions, c_T*Tf: linear -> 0.
    N  = hs.N;
    nl = size(hs.LOC, 1);

    % 1) lambda.eqnonlin = defects(:), column-major (16 x N)
    lam_eq = reshape(lambda.eqnonlin, [], N);
    V = zeros(nl*nl, N);
    for k = 1:N
        Hk = hess_loc(z(hs.LOC(:,k)), lam_eq(:,k));
        V(:,k) = Hk(:);
    end
    H = sparse(hs.I, hs.J, V(:), hs.nz, hs.nz);

    % 2) lambda.ineqnonlin = [obst_1 (N+1); obst_2 (N+1); ...]
    lam_in = reshape(lambda.ineqnonlin, N+1, hs.nb);
    P  = z(hs.IP);                                     % 3 x (N+1) positions
    dg = zeros(hs.nz, 1);
    for b = 1:hs.nb
        Pc = P - [hs.c(b,:).'; 0];                     % building centre at ground level
        h  = -hs.p*(hs.p-1) * Pc.^(hs.p-2) ./ hs.a(b,:).'.^hs.p;   % 3 x (N+1)
        dg(hs.IP) = dg(hs.IP) + h .* lam_in(:,b).';
    end
    H = H + spdiags(dg, 0, hs.nz, hs.nz);
end

% ---------- Plot helpers ----------
function draw_scene(scen)
% 3D scene: buildings, start and target (new figure, left open for further plotting).
    figure; hold on; grid on; axis equal; view(35,25);
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    Vc = [-1 -1 0; 1 -1 0; 1 1 0; -1 1 0; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1];   % unit box
    Fc = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    for b = 1:scen.n_buildings
        bd = scen.buildings(b,:);                                             % [xc yc dx dy hz]
        patch('Vertices', Vc.*[bd(3) bd(4) bd(5)] + [bd(1) bd(2) 0], 'Faces', Fc, ...
            'FaceColor',[0.8 0.2 0.2], 'FaceAlpha',0.5, 'EdgeColor','k', 'HandleVisibility','off');
    end
    xlim([scen.x_min scen.x_max]);  ylim([scen.y_min scen.y_max]);  zlim([scen.z_min scen.z_max]);
    plot3(scen.start_pos(1),  scen.start_pos(2),  scen.start_pos(3),  'gs', 'MarkerSize',12, ...
        'MarkerFaceColor','g', 'DisplayName','Start');
    plot3(scen.target_pos(1), scen.target_pos(2), scen.target_pos(3), 'p',  'MarkerSize',16, ...
        'MarkerFaceColor','y', 'MarkerEdgeColor','k', 'DisplayName','Target');
end

function plot_heading(x, y, vx, vy, color)
% Arrows (length 2 m) along the horizontal velocity, at 15 evenly spread samples.
    i = round(linspace(1, numel(x), 15));
    v = max(hypot(vx(i), vy(i)), 1e-6);
    quiver(x(i), y(i), 2*vx(i)./v, 2*vy(i)./v, 0, 'Color', color, 'HandleVisibility','off');
end
