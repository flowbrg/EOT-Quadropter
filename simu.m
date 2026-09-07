clc; clear; close all;

init_params

% Vitesse hover analytique
w_h = sqrt((params(1)*9.81) / (4*params(2)));
u_hover = [w_h; w_h; w_h; w_h];

% Test point fixe
x0 = zeros(12,1);
dx = dyn(x0, u_hover, params);
disp('dxdt au hover (doit etre ~0) :'); disp(dx)

% Simulation libre
[t, X] = ode45(@(t,x) dyn(x, u_hover, params), [0 5], x0);
figure; plot(t, X(:,3)); xlabel('t [s]'); ylabel('z [m]'); title('Altitude au hover');

% Critère de Bryson
deg2rad = pi/180;
x_max = [1; 1; 0.5; 2; 2; 2; 15*deg2rad; 15*deg2rad; 45*deg2rad; 1; 1; 1];
u_max = repmat(w_h*0.2, 4, 1); % 20% de w_hover comme variation max

Q = diag(1./x_max.^2);
R = diag(1./u_max.^2);

% Linéarisation numérique au hover
eps = 1e-6;
A = zeros(12,12);
B = zeros(12,4);

for i = 1:12
    xp = x0; xp(i) = xp(i)+eps;
    xm = x0; xm(i) = xm(i)-eps;
    A(:,i) = (dyn(xp,u_hover,params) - dyn(xm,u_hover,params)) / (2*eps);
end
for i = 1:4
    up = u_hover; up(i) = up(i)+eps;
    um = u_hover; um(i) = um(i)-eps;
    B(:,i) = (dyn(x0,up,params) - dyn(x0,um,params)) / (2*eps);
end

% LQR
K = lqr(A, B, Q, R);

% Simulation en boucle fermee
x0_pert = zeros(12,1); x0_pert(3) = 0.3; % perturbation initiale en z
ctrl = @(x) u_hover - K*(x - x0);
[t, X] = ode45(@(t,x) dyn(x, ctrl(x), params), [0 10], x0_pert);

figure;
subplot(3,1,1); plot(t, X(:,1:3)); legend('x','y','z'); ylabel('Position [m]');
subplot(3,1,2); plot(t, X(:,4:6)); legend('vx','vy','vz'); ylabel('Vitesse [m/s]');
subplot(3,1,3); plot(t, X(:,7:9)*180/pi); legend('\phi','\theta','\psi'); ylabel('Attitude [deg]');
xlabel('t [s]');