function dyn(states,commands,params)
% Etats
x       = states(1);
y       = states(2);
z       = states(3);
vx      = states(4);
vy      = states(5);
vz      = states(6);
phi     = states(1);
theta   = states(2);
psi     = states(3);
p       = states(4);
q       = states(5);
r       = states(6);

cphi    = cos(phi);
sphi    = sin(phi);
ctheta  = cos(theta);
stheta  = sin(theta);
ttheta  = stheta/ctheta;
cpsi    = cos(psi);
spsi    = sin(psi);

% Commandes
w1      = commands(1);
w2      = commands(2);
w3      = commands(3);
w4      = commands(4);

% Paramètres
m       = params(); % Masse
k_T     = params(); % Coefficient de poussée hélice
k_Q     = params(); % Coefficient de couple de réaction hélice
l       = params(); % distance des hélices au centre de gravité
Jxx     = params();
Jyy     = params();
Jzz     = params();
J       = diag([Jxx Jyy Jzz]); % Matrice d'inertie
g       = 9.81;

% Matrice de transition du repère du quadroptere dans le repere
% terrestre
R = [cphi*ctheta    cpsi*stheta*sphi-spsi*cphi      cpsi*stheta*cphi+spsi*stheta;
    spsi*ctheta    spsi*stheta*sphi+ctheta*cphi    spsi*stheta*cphi-cpsi*sphi;
    -stheta        ctheta*sphi                     ctheta*cphi];

% Matrice de dérivation des angles d'euler
W = [1  sphi*ttheta cphi*ttheta;
    0  cphi        -sphi;
    0  sphi/ctheta cphi/ctheta];

w = [p; q; r];
% eta = [phi; theta; psi]
deta = W*w;

T       = k_T*(w1^2 + w2^2 + w3^2 + w4^2);
Gphi    = l*k_T*(w4^2 - w2^2);
Gtheta  = l*k_T*(w3^2 - w1^2);
Gpsi    = k_Q*(w1^2 - w2^2 + w3^2 - w4^2);

a       = (1/m)*(R*[0;0;T]-[0;0;m*g]);
dw      = inv(J)*(-w*J*w-Jr*w*[0;0;w1-w2+w3-w4]+[Gphi; Gtheta; Gpsi]);


