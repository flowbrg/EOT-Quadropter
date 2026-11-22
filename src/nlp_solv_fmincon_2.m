%% Trajectory Optimization - 3D Quadcopter, Minimum Energy, WITH Obstacles
% Claude was used to 'clean' the code and add a few comments.
% Optimization Toolbox, problem-based approach (optimvar + prob2struct -> fmincon)
%
% 16-state model: [x y z vx vy vz phi theta psi p q r w1 w2 w3 w4]
% Controls      : U = [dw1..dw4]  angular accelerations [rad/s^2]
%
% Equivalent to old/nlp_solv_casadi_2.m (CasADi/IPOPT), with buildings.
%
%
% RUN GEN_HESS_LOC.M BEFORE!!
%
%
%  Derivatives:
%   - gradient of the objective and Jacobians of the constraints: AD (auto-reverse)
%     via prob2struct, as long as the dynamics function and optimization criterion
%     operations (code) remains supported.
%   - Hessian:
%       use_exact_hessian = true: EXACT Hessian of the Lagrangian via 'HessianFcn'
%            * objective + collocation: 41x41 local blocks (one per interval),
%              generated symbolically (gen_hess_loc.m), assembled in a concave manner;
%            * obstacles: analytical diagonal contribution;
%            * the rest (boundary conditions, c_T*Tf) is linear: 0.
%       use_exact_hessian = false: L-BFGS (reference for comparison).
% check_hessian = true: finite difference test (Hv) before solving.
%
%
% use_exact_hessian is here to show the difference in speed between using
% an estimation each step against the exact hessian.

clear; clc; close all;

load("./data/params.mat")
load("./data/scenario.mat")
addpath("model/")
addpath("data/generated_fmincon/")

%% Hessian choice
use_exact_hessian = true;    % true : HessianFcn | false : L-BFGS or BFGS
check_hessian     = true;    % validate the Hessian using finite differences before running fmincon

%% NLP Parameters
Tf_init = 10;  Tf_min = 2;  Tf_max = 40;   % [s]
N       = nlp.N;                              % intervals
nx      = nlp.nx;  nu = nlp.nu;
w_h     = drone.w_h;                        % hover speed [rad/s]

w_min  = 0.1*w_h;   w_max = 1.8*w_h;        % [rad/s]
dw_min = -500;      dw_max = 500;           % [rad/s^2]
ang_lim = deg2rad(35);                      % phi, theta bounds [rad]
c_T    = 5;                                 % Tf weight [-]

% Super-ellipsoid with even exponent p_sq (larger -> closer to the pavement,
% but NLP is less well-conditioned)
p_sq = 4; %8;

% Initial state (t=0, not initial guess/trajectory)
psi0 = 0;
x_init_bc = [scen.start_pos(:); scen.start_vel(:); 0; 0; psi0; 0; 0; 0; w_h*ones(4,1)];  % 16x1
pos_f = scen.target_pos(:);
vel_f = scen.target_vel(:);

%% Consistency Check of Dynamics (Numerical)
x_hov = [zeros(12,1); w_h*ones(4,1)];
f_hov = dyn_vec(x_hov, zeros(4,1), drone);
fprintf('Hover : |f(1:12)| = %.2e | |dw/dt| = %.2e\n', norm(f_hov(1:12)), norm(f_hov(13:16)));
assert(all(isfinite(f_hov)), 'dyn_vec returns NaN/Inf at hover');

%% Consistency of hess_loc.m with current parameters
% hess_loc.m has N, P_ref, drone, and mot: we verify that they have not changed.
if use_exact_hessian || check_hessian
    meta = load("data/generated_fmincon/hess_loc_meta.mat");
    assert(meta.N == N && meta.g0 == g0 && abs(meta.P_ref - nlp.P_ref) <= 1e-12*abs(nlp.P_ref) ...
           && isequal(meta.drone, drone) && isequal(meta.mot, mot), ...
           'data/generated_fmincon/hess_loc.m is obsolete (N, g0, drone or mot changed) : restart gen_hess_loc.m');
end

%% Optimisation variables
% Lower bounds
x_lb = [scen.x_min; scen.y_min; scen.z_min; -inf(3,1); -ang_lim; -ang_lim; -inf; -inf(3,1); w_min*ones(4,1)];
% Upper bounds
x_ub = [scen.x_max; scen.y_max; scen.z_max;  inf(3,1);  ang_lim;  ang_lim;  inf;  inf(3,1); w_max*ones(4,1)];

Tf = optimvar('Tf', 1,       'LowerBound', Tf_min,              'UpperBound', Tf_max);
X  = optimvar('X',  nx, N+1, 'LowerBound', repmat(x_lb,1,N+1),  'UpperBound', repmat(x_ub,1,N+1));
U  = optimvar('U',  nu, N+1, 'LowerBound', dw_min,              'UpperBound', dw_max);

%% Objective function / Criterion (using Simpson's quadrature)
wk   = X(13:16, 1:N);     wk1  = X(13:16, 2:N+1);
dwk  = U(:, 1:N);         dwk1 = U(:, 2:N+1);
wmid  = (wk  + wk1 )/2;
dwmid = (dwk + dwk1)/2;

Pk   = Pm_rows(wk,   dwk,   drone, mot);    % 1 x N
Pmid = Pm_rows(wmid, dwmid, drone, mot);    % 1 x N
Pk1  = Pm_rows(wk1,  dwk1,  drone, mot);    % 1 x N

obj = Tf/(6*N*nlp.P_ref) * sum(Pk + 4*Pmid + Pk1);
obj = obj + c_T*Tf;

%% Constraints
% Trapezoidal collocation defects X_{k+1} - X_k - h/2 (f_k + f_{k+1}), 16 x n.
F = dyn_vec(X, U, drone);                                          % 16 x (N+1)
defects = X(:,2:N+1) - X(:,1:N) - (Tf/(2*N))*(F(:,1:N) + F(:,2:N+1));  % 16 x N

prob = optimproblem('Objective', obj);
prob.Constraints.colloc = (defects == 0);

% Boundary conditions
prob.Constraints.init   = (X(:,1)       == x_init_bc);
prob.Constraints.pos_f  = (X(1:3,N+1)   == pos_f);
prob.Constraints.vel_f  = (X(4:6,N+1)   == vel_f);
prob.Constraints.att_f  = (X(7:8,N+1)   == 0);
prob.Constraints.rate_f = (X(10:12,N+1) == 0);
prob.Constraints.rot_f  = (X(13:16,N+1) == w_h*ones(4,1));

% Obstacles (AABB buildings, smooth super-ellipsoid formulation):
% s_b(r) = ((x-xc)/ax)^p + ((y-yc)/ay)^p + (z/az)^p >= 1
% where ax = dx + offset, ay = dy + offset, az = hz + offset (z >= 0: center at ground level).
% One (1 x N+1) vector constraint per building, applied to all nodes.
for b = 1:scen.n_buildings
    xc = scen.buildings(b,1);  yc = scen.buildings(b,2);
    ax = scen.buildings(b,3) + scen.ecart;
    ay = scen.buildings(b,4) + scen.ecart;
    az = scen.buildings(b,5) + scen.ecart;

    sb = ((X(1,:) - xc)/ax).^p_sq ...
       + ((X(2,:) - yc)/ay).^p_sq ...
       + ( X(3,:)      /az).^p_sq;

    prob.Constraints.(sprintf('obst_%d', b)) = (sb >= 1);
end

%% Initial guess
% Position: smoothstep xy (zero speed at the ends) + altitude profile
% in "plateau" mode: rapid ascent, cruising above the tallest building,
% final descent.
tau   = linspace(0, 1, N+1);
sprof = 3*tau.^2 - 2*tau.^3;
dsdt  = (6*tau - 6*tau.^2) / Tf_init;

z_cruise = max(scen.buildings(:,5)) + 2*scen.ecart;   % [m]
if z_cruise >= scen.z_max
    error('z_cruise (%.1f m) >= z_max (%.1f m)', z_cruise, scen.z_max);
end
sstep  = @(a) 3*min(max(a,0),1).^2 - 2*min(max(a,0),1).^3;
z_prof = z_cruise*(sstep(tau/0.15) - sstep((tau-0.85)/0.15));

p0 = scen.start_pos(:);  pf = scen.target_pos(:);

X_init = zeros(nx,N+1);
X_init(1:3,:)   = p0 + (pf-p0)*sprof;
X_init(3,:)     = X_init(3,:) + z_prof;
X_init(4:6,:)   = (pf-p0)*dsdt;
X_init(6,:)     = X_init(6,:) + gradient(z_prof, Tf_init/N);
X_init(9,:)     = psi0;
X_init(13:16,:) = w_h*ones(4,N+1);

x0.Tf = Tf_init;
x0.X  = X_init;
x0.U  = zeros(nu,N+1);

%% Problem to structure
genDir = fullfile(pwd,'data/generated_fmincon');
if ~exist(genDir,'dir')
    mkdir(genDir);
end
addpath(genDir)

solverProb = prob2struct(prob, x0, ...
    'ObjectiveDerivative',  'auto-reverse', ...
    'ConstraintDerivative', 'auto-reverse', ...
    'FileLocation', genDir, ...
    'ObjectiveFunctionName', 'fmincon_objective_2', ...
    'ConstraintFunctionName', 'fmincon_constraints_2');

%% Exact hessian : pre-calculations
% Variables order in z, given by varindex.
% This part prevents a lot of unnecessary computations
idx = varindex(prob);
nz  = 1 + (nx+nu)*(N+1);
assert(numel(solverProb.x0) == nz, 'Expected z dimension');

IX   = reshape(idx.X, nx, N+1);              % indices of X(:,k) in z
IU   = reshape(idx.U, nu, N+1);              % indices of U(:,k) in z
nloc = 1 + 2*(nx+nu);                        % 41 : zl = [Tf; x_k; u_k; x_{k+1}; u_{k+1}]
LOC  = zeros(nloc, N);
for k = 1:N
    LOC(:,k) = [idx.Tf; IX(:,k); IU(:,k); IX(:,k+1); IU(:,k+1)];
end

hs.N   = N;   hs.nx = nx;   hs.nz = nz;
hs.LOC = LOC;
% Indices (I, J) of each local block, in column-major order of Hk(:):
hs.I   = reshape(repmat(reshape(LOC, nloc, 1, N), [1 nloc 1]), [], 1);
hs.J   = reshape(repmat(reshape(LOC, 1, nloc, N), [nloc 1 1]), [], 1);
% Obstacles
hs.IP  = IX(1:3,:);                          % indices of (x,y,z) at each node : 3 x (N+1)
hs.nb  = scen.n_buildings;
hs.c   = scen.buildings(:,1:2);              % centers (xc, yc)
hs.a   = scen.buildings(:,3:5) + scen.ecart; % semi-axes (ax, ay, az)
hs.p   = p_sq;

Hfun = @(z, lambda) hess_lagr(z, lambda, hs);

%% Hessian validation (finite difference over gradient of Lagrangian)
if check_hessian
    err_hess = check_hess_fd(solverProb, Hfun, hs);
    if err_hess > 1e-4
        error(['Exact Hessian inconsistent with AD gradients (relative error %.1e). ' ...
               'See details above: the erroneous case indicates the source ' ...
               '(collocation: order/sign of lambda.eqnonlin; obstacles: order/sign of lambda.ineqnonlin).'], err_hess);
    end
end

%% Fmincon options
options = optimoptions('fmincon', ...
    'Algorithm',                'interior-point', ...
    'EnableFeasibilityMode',    false, ...
    'SpecifyObjectiveGradient', true, ...
    'SpecifyConstraintGradient',true, ...
    'ScaleProblem',             true, ...
    'MaxIterations',            3000, ...
    'MaxFunctionEvaluations',   1e7, ...
    'OptimalityTolerance',      1e-5, ...
    'ConstraintTolerance',      1e-5, ...
    'StepTolerance',            1e-3, ...
    'Display',                  'iter');

if use_exact_hessian
    options.HessianFcn = Hfun;                 % Exact hessian of Lagrangian
else
    options.HessianApproximation = 'lbfgs';    % reference
end

solverProb.options = options;

%% Solve
tic
[z_sol, fval, exitflag, output, lambda, grad, hessian] = ...
    fmincon(solverProb);
t_solve = toc;

%% Extraction (same as old/nlp_solv_casadi_2.m and nlp_solv_fmincon.m)
% idx = varindex(prob) is calculated earlier (pre-computations for the Hessian).

Tf_sol = z_sol(idx.Tf);
X_sol  = reshape(z_sol(idx.X), nx, N+1);
U_sol  = reshape(z_sol(idx.U), nu, N+1);

t_sol  = linspace(0, Tf_sol, N+1);

% Electrical energy [J] (U_sol = accelerations, speeds in X_sol(13:16,:))
Wn  = X_sol(13:16,:);
E_P = Pm_rows(Wn(:,1:N),               U_sol(:,1:N),               drone, mot) ...
    + 4*Pm_rows((Wn(:,1:N)+Wn(:,2:N+1))/2, (U_sol(:,1:N)+U_sol(:,2:N+1))/2, drone, mot) ...
    + Pm_rows(Wn(:,2:N+1),             U_sol(:,2:N+1),             drone, mot);
E_sol = Tf_sol/(6*N) * sum(E_P);

% A posteriori obstacle check: min_k s_b(r_k) must be >= 1
for b = 1:scen.n_buildings
    xc = scen.buildings(b,1);  yc = scen.buildings(b,2);
    ax = scen.buildings(b,3) + scen.ecart;
    ay = scen.buildings(b,4) + scen.ecart;
    az = scen.buildings(b,5) + scen.ecart;
    sb_sol = ((X_sol(1,:)-xc)/ax).^p_sq + ((X_sol(2,:)-yc)/ay).^p_sq + (X_sol(3,:)/az).^p_sq;
    fprintf('Building %d : min s_b = %.4f (>= 1 required)\n', b, min(sb_sol));
end

fprintf('Optimal Tf : %.2f s\n', Tf_sol);
fprintf('Energy     : %.1f J  (hover during Tf : %.1f J)\n', E_sol, nlp.P_ref*Tf_sol);
fprintf('exitflag = %d | iterations = %d | constr. viol. = %.2e\n', ...
        exitflag, output.iterations, output.constrviolation);
fprintf('Hessian  : %s | solving time = %.1f s (%.3f s/iter)\n', ...
        ternary(use_exact_hessian, 'exact (HessianFcn)', 'L-BFGS'), t_solve, t_solve/max(output.iterations,1));

save('sols/sol_fmincon_2.mat', 'X_sol', 'U_sol', 't_sol', 'Tf_sol', 'E_sol');

%% Local functions

function s = ternary(c, a, b)
    if c, s = a; else, s = b; end
end

function H = hess_lagr(z, lambda, hs)
% HESS_LAGR  Hessian of the Lagrangian  L = f + lam_ineq'*c + lam_eq'*ceq
%            (fmincon interior-point convention: + sign for both types)
%
% Contributions:
% 1) objective + collocation: sum of the 41x41 local blocks from model/hess_loc.m,
%    assembled in a sparse manner (sparse sum due to overlaps between neighboring intervals);
% 2) constraints: c_b = 1 - s_b(r_k), DIAGONAL Hessian (only the pair x_k, y_k, z_k):
%    d^2 c_b / dr_i^2 = -p(p-1) (r_i - c_i)^(p-2) / a_i^p
% 3) Boundary conditions, c_T*Tf: linear -> 0.
    N    = hs.N;
    nloc = size(hs.LOC, 1);

    % 1) local blocks (lambda.eqnonlin = defects(:), major-column: 16 x N)
    lam_eq = reshape(lambda.eqnonlin, hs.nx, N);
    V = zeros(nloc*nloc, N);
    for k = 1:N
        Hk = hess_loc(z(hs.LOC(:,k)), lam_eq(:,k));    % model/hess_loc.m
        V(:,k) = Hk(:);
    end
    H = sparse(hs.I, hs.J, V(:), hs.nz, hs.nz);

    % 2) constraints (lambda.ineqnonlin = [constraint_1 (N+1); constraint_2 (N+1); ...])
    lam_in = reshape(lambda.ineqnonlin, N+1, hs.nb);   % (N+1) x nb
    P  = z(hs.IP);                                     % 3 x (N+1) : positions
    dg = zeros(hs.nz, 1);
    for b = 1:hs.nb
        Pc = P - [hs.c(b,:).'; 0];                     % center of the building (z_c = 0)
        a  = hs.a(b,:).';                              % 3 x 1
        h  = -hs.p*(hs.p-1) * Pc.^(hs.p-2) ./ a.^hs.p; % 3 x (N+1)
        dg(hs.IP) = dg(hs.IP) + h .* lam_in(:,b).';
    end
    H = H + spdiags(dg, 0, hs.nz, hs.nz);
end

function g = lagr_grad(sp, z, lam)
% Lagrangian gradient based on the AD gradients generated by prob2struct:
% grad L = grad f + gc*lam_ineq + gceq*lam_eq   (gc, gceq: nz × m)
    [~, gf]         = sp.objective(z);
    [~, ~, gc, gceq] = sp.nonlcon(z);
    g = gf(:) + gc*lam.ineqnonlin + gceq*lam.eqnonlin;
end

function err_max = check_hess_fd(sp, Hfun, hs)
% Compares H*v to the centered finite difference of the Lagrangian gradient.
% Four cases isolate the source of an error:
%   - objective alone (lam = 0)
%   - collocation alone
%   - obstacles alone
%   - all together
% Expected: relative errors ~1e-7 or less. An O(1) error indicates a problem
% with the sign or order of the multipliers, or an erroneous local block.
% This function was AI-generated
    rng(0);
    z0 = sp.x0(:);   nz = numel(z0);
    z  = z0 + 0.01*(1 + abs(z0)).*randn(nz,1);         % Generic percentage point (non-trivial derivatives)

    [~, ~, gc, gceq] = sp.nonlcon(z);
    ni = size(gc, 2);   ne = size(gceq, 2);
    assert(ne == hs.nx*hs.N,       'lambda.eqnonlin : %d entries, %d expected (collocation ?)', ne, hs.nx*hs.N);
    assert(ni == (hs.N+1)*hs.nb,   'lambda.ineqnonlin : %d entries, %d expected (obstacles ?)', ni, (hs.N+1)*hs.nb);

    noms = {'objective function (lam = 0)', 'collocation', 'obstacles', 'all together'};
    err_max = 0;
    fprintf('\n--- Hessian validation (Hv vs finite differences) ---\n');
    for c = 1:4
        lam.eqnonlin   = zeros(ne, 1);
        lam.ineqnonlin = zeros(ni, 1);
        if c == 2 || c == 4, lam.eqnonlin   = randn(ne, 1); end
        if c == 3 || c == 4, lam.ineqnonlin = rand(ni, 1);  end

        H    = Hfun(z, lam);
        asym = norm(H - H.', 'fro') / max(norm(H, 'fro'), realmin);

        e = 0;  h = 1e-6;
        for t = 1:3
            v = randn(nz,1).*(1 + abs(z));  v = v/norm(v);
            Hv_fd = (lagr_grad(sp, z + h*v, lam) - lagr_grad(sp, z - h*v, lam)) / (2*h);
            e = max(e, norm(H*v - Hv_fd) / max(norm(Hv_fd), realmin));
        end
        fprintf('%-26s : relative error Hv = %.2e | asymmetry = %.1e | nnz(H) = %d\n', ...
                noms{c}, e, asym, nnz(H));
        err_max = max(err_max, e);
    end
    fprintf('--> max error = %.2e (threshold 1e-4)\n\n', err_max);
end