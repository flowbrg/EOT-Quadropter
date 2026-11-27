%% gen_hess_loc.m
% SYMBOLIC generation of the local block of the Hessian of the Lagrangian.
% Run only only (and rerun if N, drone, or mot change).
%
% Outputs:
% model/hess_loc.m        : [Hloc] = hess_loc(zl, lam), 41x41
% model/hess_loc_meta.mat : parameters used in hess_loc
%
% Local Lagrangian for interval k (local variables zl = [Tf; x_k; u_k; x_{k+1}; u_{k+1}]):
% L_k = f_k(zl) + lam' * default_k(zl)
% where f_k is the Simpson contribution of interval k to the objective function and
% defaut_k is the trapezoidal colocation error (16 components).
% The global Hessian is the SUM of the local blocks, which is NOT done is this function.
% The linear term c_T*Tf has a zero Hessian.

N  = nlp.N;
nx = nlp.nx; nu = nlp.nu;
P_ref = nlp.P_ref;

%% Local symbolic variables
syms Tf real
xa  = sym('xa',  [nx 1], 'real');    % x_k
ua  = sym('ua',  [nu 1], 'real');    % u_k
xb  = sym('xb',  [nx 1], 'real');    % x_{k+1}
ub  = sym('ub',  [nu 1], 'real');    % u_{k+1}
lam = sym('lam', [nx 1], 'real');    % multipliers for incorrect collocation

zl = [Tf; xa; ua; xb; ub];           % 41 local variables

%% Trapezoidal collocation defects
F0 = dyn_vec(xa, ua, drone);
F1 = dyn_vec(xb, ub, drone);
d  = xb - xa - (Tf/(2*N))*(F0 + F1);

%% Contribution of the local interval using Simpson's quadrature
wa = xa(13:16);   wb = xb(13:16);
dwa = ua;         dwb = ub;
fk = Tf/(6*N*P_ref) * ( Pm_rows(wa,         dwa,           drone, mot) ...
                      + 4*Pm_rows((wa+wb)/2, (dwa+dwb)/2,  drone, mot) ...
                      +   Pm_rows(wb,        dwb,          drone, mot) );

%% Hessian of Lagrangian
tic
Lloc = fk + lam.'*d;
Hloc = hessian(Lloc, zl);            % 41 x 41, symmetrical
fprintf('Symbolic Hessian calculated in %.1f s\n', toc);

tic
matlabFunction(Hloc, 'File', 'model/hess_loc', 'Vars', {zl, lam}, 'Optimize', true);
fprintf('model/hess_loc.m genere en %.1f s\n', toc);
rehash; clear hess_loc

save('data/generated_fmincon/hess_loc_meta.mat', 'N', 'g0', 'P_ref', 'drone', 'mot');
fprintf('Metadatas saved (N=%d, g0=%.2f, P_ref=%.4f W)\n', N, g0, P_ref);
