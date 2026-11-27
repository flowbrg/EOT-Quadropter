%% test_motor_reduction.m
%
%   MOVE TO SRC/
%
% This script was AI-generated using Claude Sonnet 4.6
%
% Quantitative test: "mechanics-only" planner vs full motor+battery model.
% Level 1: planner model (dyn.m, input = w*(t)), comparison baseline.
% Level 2: full model, open-loop control w_ref(t) = w*(t).
%   A : ideal battery (V=E_ocv, R_int=0), without acceleration FF  -> motor-only error
%   B : ideal battery, with acceleration FF                        -> predicted e = 0 (validation)
%   C : real battery, without acceleration FF                        -> current configuration
clear; clc; close all;

%% 0. Data ---------------------------------------------------------------------
load('./data/params.mat');                        % drone, batt, mot, SoC0
S  = load('./data/scenario.mat');  scen = S.scen;
Rs = load('./sols/sol_casadi.mat');
X_sol = Rs.X_sol;  U_sol = Rs.U_sol;              % 16x(N+1), 4x(N+1)
t_sol = Rs.t_sol(:).';  Tf = Rs.Tf_sol;
W = X_sol(13:16,:);  dW = U_sol;  wh = drone.w_h;
assert(abs(drone.k_Q - mot.d) < 1e-15, 'k_Q (planner) ~= d (motor) : static bias');
assert(numel(t_sol) == size(X_sol,2));

Kc  = mot.KE*batt.E_ocv/mot.R;
cw  = @(w) mot.KE^2/mot.R + 2*mot.d*w + Kc*mot.Kp;     % a(w) + K*Kp
tau_eff = mot.Jr/cw(wh);
Ltraj = norm(scen.target_pos(:) - scen.start_pos(:));   % start-to-target distance [m]
fprintf('tau_eff(w_h) = %.2f ms | start-to-target distance = %.1f m\n', 1e3*tau_eff, Ltraj);

% Smooth reference w*(t) and its exact derivative (pchip)
pp = pchip(t_sol, W);
[brk,cf,~,kord,dm] = unmkpp(pp);
dpp   = mkpp(brk, cf(:,1:kord-1).*repmat(kord-1:-1:1,size(cf,1),1), dm);
wref  = @(t) ppval(pp,t);   dwref = @(t) ppval(dpp,t);
tout  = linspace(0,Tf,2001);
ctx = struct('wref',wref,'dwref',dwref,'drone',drone,'batt',batt,'tout',tout, ...
             'x0',[X_sol(1:16,1); SoC0; zeros(4,1)]);

%% 1. Feasibility: (w*, u*) within reachable acceleration envelope ------------
Voc0 = polyval(batt.Pol_ocv, SoC0) + log(SoC0)*SoC0;
ws   = linspace(0.1*wh, 1.8*wh, 300);                  % same bounds as the NLP
amax = @(V) (mot.KE*(V-mot.KE*ws)/mot.R - mot.d*ws.^2)/mot.Jr;
amin = (-mot.KE^2*ws/mot.R - mot.d*ws.^2)/mot.Jr;
ok = @(am) W>=min(ws) & W<=max(ws) & dW<=interp1(ws,am,W,'linear',Inf) ...
           & dW>=interp1(ws,amin,W,'linear',-Inf);
fprintf('max|u*| = %.1f rad/s^2 | nodes outside envelope : V=E_ocv %.1f%% | V=Voc(SoC0)=%.2f V %.1f%%\n', ...
        max(abs(dW(:))), 100*mean(~ok(amax(batt.E_ocv)),'all'), Voc0, 100*mean(~ok(amax(Voc0)),'all'));
fprintf('Voc(SoC0) = %.2f V vs E_ocv = %.2f V (FF uses E_ocv)\n', Voc0, batt.E_ocv);

%% 2. Level 1 vs NLP (discretization + parameter consistency) ------------------
tight = odeset('RelTol',1e-10,'AbsTol',1e-10);
[~,X1] = ode45(@(t,x) dyn(x,wref(t),drone), tout, X_sol(1:12,1), tight);
Xnlp = interp1(t_sol, X_sol(1:12,:).', tout, 'pchip');
fprintf('Level 1 vs NLP : max pos err = %.3e m | end vs target = %.3e m (%.1f%% of Ltraj)\n', ...
        max(sqrt(sum((Xnlp(:,1:3)-X1(:,1:3)).^2,2))), norm(X1(end,1:3)-scen.target_pos(:).'), ...
        100*norm(X1(end,1:3)-scen.target_pos(:).')/Ltraj);

%% 3. Level 2 vs Level 1: cases A, B, C ----------------------------------------
cases = struct('name',{'A ideal, no FFacc','B ideal, FFacc','C real, no FFacc'}, ...
               'ib',{true,true,false}, 'ff',{false,true,false});
res = cell(1,3);
fprintf('\n%-20s %11s %11s %12s %11s %9s %11s\n','case','errPos max','errPos end', ...
        'errAtt max','max|e|/wh','Dc sat','min Vt/Eocv');
fprintf('%-20s %11s %11s %12s %11s %9s %11s\n','','[m]','[m]','[deg]','[%]','[%]','[-]');
for ic = 1:3
    X2  = run_full(ctx, cases(ic), mot);
    pe  = sqrt(sum((X2(:,1:3)-X1(:,1:3)).^2,2));
    ae  = rad2deg(max(abs(X2(:,7:9)-X1(:,7:9)),[],'all'));
    es  = wref(tout).' - X2(:,13:16);
    sat = 0;  vr = inf;
    for i = 1:numel(tout)
        [Dc,~,Vt] = motor_stage(tout(i), X2(i,:).', ctx, cases(ic), mot);
        sat = sat + any(Dc<=0 | Dc>=1);  vr = min(vr, Vt/batt.E_ocv);
    end
    res{ic} = struct('pe',pe);
    fprintf('%-20s %11.3e %11.3e %12.3f %11.3f %9.1f %11.3f\n', cases(ic).name, max(pe), pe(end), ...
            ae, 100*max(abs(es),[],'all')/wh, 100*sat/numel(tout), vr);
end
fprintf('Max error case A relative to Ltraj : %.3f %%\n', 100*max(res{1}.pe)/Ltraj);

figure('Name','Position error (level 2 - level 1)');
semilogy(tout, max(res{1}.pe,eps), tout, max(res{2}.pe,eps), tout, max(res{3}.pe,eps));
grid on; legend('A','B','C','Location','best');
xlabel('t [s]'); ylabel('||p_{full} - p_{plan}|| [m]');

%% 4. Sweep of tau (Jr): expected log-log slope = 1 (case A) ------------------
kap = [0.25 0.5 1 2 4];  emax = zeros(size(kap));  teff = zeros(size(kap));
for j = 1:numel(kap)
    ml = mot;  ml.Jr = kap(j)*mot.Jr;
    Xs = run_full(ctx, cases(1), ml);
    emax(j) = max(sqrt(sum((Xs(:,1:3)-X1(:,1:3)).^2,2)));
    teff(j) = ml.Jr/(mot.KE^2/mot.R + 2*mot.d*wh + Kc*mot.Kp);
end
pfit = polyfit(log10(teff), log10(emax), 1);
fprintf('\nSweep : log-log slope of max error vs tau_eff = %.2f (expected ~1)\n', pfit(1));
figure('Name','Sweep'); loglog(teff,emax,'o-'); grid on;
xlabel('\tau_{eff} [s]'); ylabel('max position error [m]');

%% ----------------------------- local functions ---------------------------------
function [Dc,dz,Vterm,Ib,dw] = motor_stage(t,X,ctx,opt,motl)
    batt = ctx.batt;
    if opt.ib
        batt.R_int = 0;  Voc = batt.E_ocv;
    else
        Voc = polyval(batt.Pol_ocv,X(17)) + log(X(17))*X(17);
    end
    [Dc,dz] = dutycycle(ctx.wref(t), X(13:16), X(18:21), batt, motl);
    if opt.ff   % Dc_ff += R*Jr/(KE*E) * dw_ref/dt
        Dc = min(1,max(Dc + motl.R*motl.Jr/(motl.KE*batt.E_ocv)*ctx.dwref(t),0));
    end
    Vterm = (motl.R*Voc + batt.R_int*motl.KE*sum(Dc.*X(13:16))) ...
            /(motl.R + batt.R_int*sum(Dc.^2));
    [Ib,dw] = dyn_bldcm(Dc,Voc,X(13:16),batt,motl);
end

function dX = dyn_full(t,X,ctx,opt,motl)
    [~,dz,~,Ib,dw] = motor_stage(t,X,ctx,opt,motl);
    dX = [dyn(X(1:12),X(13:16),ctx.drone); dw; -Ib/ctx.batt.C; dz];
end

function X = run_full(ctx,opt,motl)
    o = odeset('RelTol',1e-8,'AbsTol',1e-9);
    [~,X] = ode15s(@(t,x) dyn_full(t,x,ctx,opt,motl), ctx.tout, ctx.x0, o);
end