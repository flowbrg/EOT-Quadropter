%% compare_models.m
% Comparison of dyn (mechanical only) and dyn_uav (mechanical + BLDC motors
% + battery). Both models take the 4 motor speeds as input.
%
% Needs: params.mat, dyn.m, dyn_uav.m, dutycycle.m, dyn_bldcm.m,
%        Control System Toolbox. Optimization Toolbox (fsolve) is used for
%        trimming the controller integrators, with a simulation fallback.
%
% Method:
%   1) Trim at hover (U = w_h on all motors)
%   2) Numerical linearisation of both models
%   3) Virtual inputs (collective, roll, pitch, yaw) via a mixing matrix
%   4) Frequency analysis: Bode, Nyquist, singular values, poles
%   5) Nonlinear time simulations: step and pulse, with linear overlay
%   6) Step metrics of the motor speed vs. first-order model mot.tau_m

clear; clc; close all;
load('params.mat');                     % drone, batt, mot, xi0, w0_motor, SoC0
wh = drone.w_h;

f1 = @(x,u) dyn(x, u, drone);                    % 12 states
f2 = @(x,u) dyn_uav(x, u, drone, batt, mot);     % 21 states
n1 = 12;  n2 = 21;

%% 1) Hover trim ---------------------------------------------------------
u0  = wh*ones(4,1);
x1h = zeros(n1,1);  x1h(1:3) = xi0;
x2h = [xi0; zeros(9,1); w0_motor; SoC0; zeros(4,1)];
x2h(13:16) = u0;                                  % motors at w_h

% Integrator states z (18:21): find z such that motor accelerations vanish
z0 = zeros(4,1);
try
    opts = optimoptions('fsolve','Display','off','TolFun',1e-12,'TolX',1e-12);
    z0 = fsolve(@(z) trim_res(z,x2h,u0,f2), z0, opts);
catch
    warning('fsolve unavailable/failed: trimming by simulation.');
    [~,Xs] = ode15s(@(t,x) f2(x,u0), [0 20], x2h);
    z0 = Xs(end,18:21).';
end
x2h(18:21) = z0;
d = f2(x2h,u0);
fprintf('Trim residual |dw| = %.3e rad/s^2, |dz| = %.3e\n', norm(d(13:16)), norm(d(18:21)));

%% 2) Linearisation ------------------------------------------------------
[A1,B1] = jac(f1, x1h, u0);
[A2,B2] = jac(f2, x2h, u0);

%% 3) Virtual inputs and outputs ----------------------------------------
%         collective  roll   pitch  yaw      (rows = motors 1..4)
M = [      1           0     -1      1;
           1          -1      0     -1;
           1           0      1      1;
           1           1      0     -1];
inNames  = {'collective','roll','pitch','yaw'};
outNames = {'z','\phi','\theta','\psi'};
idx = [3 7 8 9];                                   % z, phi, theta, psi

C1 = zeros(4,n1);  C1(sub2ind(size(C1),1:4,idx)) = 1;
C2 = zeros(4,n2);  C2(sub2ind(size(C2),1:4,idx)) = 1;
G1 = ss(A1, B1*M, C1, 0);                          % mechanical only
G2 = ss(A2, B2*M, C2, 0);                          % full system
Gw = ss(A2, B2*M, [zeros(4,12) eye(4) zeros(4,5)], 0);   % u -> actual w

G1.InputName = inNames; G2.InputName = inNames;

fprintf('\nPoles of dyn    :\n'); disp(sort(pole(G1)));
fprintf('Poles of dyn_uav:\n');  disp(sort(pole(G2)));

%% 4) Frequency analysis -------------------------------------------------
w = logspace(-1, 3, 800);

% 4a) Bode, diagonal channels
figure('Name','Bode - diagonal channels','Color','w');
for i = 1:4
    H1 = squeeze(freqresp(G1(i,i), w));  H2 = squeeze(freqresp(G2(i,i), w));
    subplot(2,4,i);
    semilogx(w,20*log10(abs(H1)),'b',w,20*log10(abs(H2)),'r--','LineWidth',1.3);
    grid on; title([inNames{i} ' \rightarrow ' outNames{i}]);
    if i==1, ylabel('Magnitude [dB]'); end
    subplot(2,4,4+i);
    semilogx(w,rad2deg(unwrap(angle(H1))),'b',w,rad2deg(unwrap(angle(H2))),'r--','LineWidth',1.3);
    grid on; xlabel('\omega [rad/s]');
    if i==1, ylabel('Phase [deg]'); legend('dyn','dyn\_uav','Location','best'); end
end

% 4b) Nyquist, diagonal channels
figure('Name','Nyquist - diagonal channels','Color','w');
for i = 1:4
    subplot(2,2,i);
    nyquist(G1(i,i), G2(i,i), {0.1, 1e3}); grid on;
    title([inNames{i} ' \rightarrow ' outNames{i}]);
    if i==1, legend('dyn','dyn\_uav'); end
end

% 4c) Singular values (4x4 MIMO)
figure('Name','Singular values','Color','w');
sv1 = sigma(G1,w);  sv2 = sigma(G2,w);
subplot(1,2,1);
semilogx(w,20*log10(sv1.'),'b'); hold on;
semilogx(w,20*log10(sv2.'),'r--'); grid on;
xlabel('\omega [rad/s]'); ylabel('\sigma [dB]');
title('G: virtual inputs \rightarrow [z \phi \theta \psi]  (blue: dyn, red: dyn\_uav)');
subplot(1,2,2);
semilogx(w,20*log10(sv2.'./sv1.'),'LineWidth',1.2); grid on;
xlabel('\omega [rad/s]'); ylabel('\sigma_i(G_{uav}) / \sigma_i(G_{dyn}) [dB]');
title('Effect of motors + battery on each singular value');

% 4d) Motor-only response: w_ref -> w, vs. first-order model
figure('Name','Motor dynamics','Color','w');
Hm = squeeze(freqresp(Gw(1,1), w));
H1st = 1./(1j*w*mot.tau_m + 1);
subplot(2,1,1);
semilogx(w,20*log10(abs(Hm)),'r',w,20*log10(abs(H1st)),'k--','LineWidth',1.3); grid on;
ylabel('[dB]'); title('Collective: w_{ref} \rightarrow w (actual)'); legend('dyn\_uav','1^{st} order \tau_m');
subplot(2,1,2);
semilogx(w,rad2deg(unwrap(angle(Hm))),'r',w,rad2deg(unwrap(angle(H1st))),'k--','LineWidth',1.3);
grid on; xlabel('\omega [rad/s]'); ylabel('[deg]');

%% 5) Nonlinear time simulations ----------------------------------------
Tend = 1;%3;
t = linspace(0,Tend,3001).';
t_on = 0.1;  t_pulse = 0.2;
amp = [0.01 0.002 0.002 0.01]*wh; %[0.01 0.005 0.005 0.01]*wh;                 % per virtual input [rad/s]
odeopt = odeset('RelTol',1e-8,'AbsTol',1e-9,'MaxStep',2e-3);

kinds = {'step','pulse'};
for k = 1:2
    for ch = 1:4
        if k==1
            s = @(tt) double(tt >= t_on);
        else
            s = @(tt) double(tt >= t_on && tt < t_on+t_pulse);
        end
        uf = @(tt) u0 + M(:,ch)*amp(ch)*s(tt);

        [~,X1] = ode15s(@(tt,x) f1(x,uf(tt)), t, x1h, odeopt);
        [~,X2] = ode15s(@(tt,x) f2(x,uf(tt)), t, x2h, odeopt);

        Ulin = amp(ch)*arrayfun(s,t);             % deviation of virtual input
        Y1l = lsim(G1(:,ch), Ulin, t);  Y2l = lsim(G2(:,ch), Ulin, t);

        figure('Name',sprintf('%s - %s',kinds{k},inNames{ch}),'Color','w');
        for o = 1:4
            subplot(2,4,o);
            plot(t,X1(:,idx(o)),'b',t,X2(:,idx(o)),'r--','LineWidth',1.3); hold on;
            plot(t,x1h(idx(o))+Y1l(:,o),'b:',t,x2h(idx(o))+Y2l(:,o),'r:');
            grid on; title(outNames{o});
            if o==1, legend('dyn','dyn\_uav','lin dyn','lin uav','Location','best'); end
            subplot(2,4,4+o);
            plot(t,X2(:,idx(o))-X1(:,idx(o)),'k','LineWidth',1.2); grid on;
            xlabel('t [s]'); if o==1, ylabel('uav - dyn'); end
        end
        sgtitle(sprintf('%s on %s channel (amp = %.2f rad/s)',kinds{k},inNames{ch},amp(ch)));

        dmax = max(abs(X2(:,idx)-X1(:,idx)));
        fprintf('%-5s %-10s max|uav-dyn| : z=%.3e  phi=%.3e  theta=%.3e  psi=%.3e\n', ...
                kinds{k}, inNames{ch}, dmax);
    end
end

% Motor speeds and SoC during the collective step
uf = @(tt) u0 + M(:,1)*amp(1)*double(tt>=t_on);
[~,X2] = ode15s(@(tt,x) f2(x,uf(tt)), t, x2h, odeopt);
figure('Name','Motor speed / SoC (collective step)','Color','w');
subplot(2,1,1); plot(t,X2(:,13:16),'LineWidth',1.2); hold on;
plot(t,arrayfun(@(tt) uf(tt)'*[1;0;0;0],t),'k--'); grid on;
ylabel('w_i [rad/s]'); title('Actual motor speeds vs. reference');
subplot(2,1,2); plot(t,X2(:,17),'LineWidth',1.2); grid on;
xlabel('t [s]'); ylabel('SoC [-]');

%% 6) Step metrics of the motor speed -----------------------------------
[ym,tm] = step(Gw(1,1)*amp(1), 0.5);
yf = ym(end);
t63 = tm(find(ym >= 0.632*yf,1));
si  = stepinfo(ym,tm,yf);
fprintf('\nMotor (collective step, w_ref -> w):\n');
fprintf('  rise time (10-90%%) = %.4f s | settling (2%%) = %.4f s | t63 = %.4f s\n', ...
        si.RiseTime, si.SettlingTime, t63);
fprintf('  mot.tau_m (1st order model) = %.4f s\n', mot.tau_m);
fprintf('  DC gain w/w_ref = %.4f (1 expected if the PI loop works)\n', yf/amp(1));

%% Local functions -------------------------------------------------------
function r = trim_res(z, x, u, f)
    x(18:21) = z;
    d = f(x,u);
    r = d(13:16);
end

function [A,B] = jac(f, x0, u0)
    nx = numel(x0); nu = numel(u0);
    A = zeros(nx,nx); B = zeros(nx,nu);
    for j = 1:nx
        h = 1e-6*max(1,abs(x0(j)));
        xp = x0; xp(j) = xp(j)+h;  xm = x0; xm(j) = xm(j)-h;
        A(:,j) = (f(xp,u0)-f(xm,u0))/(2*h);
    end
    for j = 1:nu
        h = 1e-6*max(1,abs(u0(j)));
        up = u0; up(j) = up(j)+h;  um = u0; um(j) = um(j)-h;
        B(:,j) = (f(x0,up)-f(x0,um))/(2*h);
    end
end