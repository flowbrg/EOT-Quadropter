%% linearize_hover.m -- Linearisation de sim_aug au hover + stabilite du LQR
% Prerequis : Simulink Control Design, Control System Toolbox,
%             params.mat (drone, battery, mot), dyn.m utilisable (voir NOTE 1).
clc; clear; close all;

load('params.mat');
mdl   = 'sim_aug';                       % nom reel du modele (cf. erreurs precedentes)
g     = 9.81;
w_h   = sqrt(drone.m*g/(4*drone.k_T));   % [rad/s]
u_eq  = w_h*ones(4,1);                   % [4x1]
xi0   = [0;0;0];  SoC_0 = 1;  w0_motor = u_eq;   % lus par le modele (workspace de base)

load_system(mdl);
% L'option "Input" (LoadExternalInput) du modele est cochee : on la coupe
% pendant findop/linearize, puis on la restaure.
ext0 = get_param(mdl,'LoadExternalInput');
set_param(mdl,'LoadExternalInput','off');
cleanupObj = onCleanup(@() set_param(mdl,'LoadExternalInput',ext0));

%% --- Point d'operation (H5) ---
opspec = operspec(mdl);

% Liste des etats : identifiez celui de l'integrateur de SoC (bloc Battery).
for i = 1:numel(opspec.States)
    fprintf('  etat %2d : %s\n', i, opspec.States(i).Block);
end
iSoC = 2;   % <-- A RENSEIGNER apres lecture de la liste ci-dessus
assert(~isnan(iSoC), 'Renseigner iSoC (indice de l''etat SoC) puis relancer.');

opspec.Inputs(1).Known = true(4,1);     % [4x1] les 4 composantes de w_ref sont imposees
opspec.Inputs(1).u     = u_eq;          % [4x1] rad/s
opspec.States(iSoC).Known       = true;
opspec.States(iSoC).x           = SoC_0;
opspec.States(iSoC).SteadyState = false;       % dSoC/dt ~= 0

[op, opreport] = findop(mdl, opspec, findopOptions('DisplayReport','iter'));
%assert(opreport.TerminationString(1) ~= 'o' || true, ''); %#ok<NOP>
disp(opreport.TerminationString);

%% --- Linearisation du modele complet (H6) ---
G = linearize(mdl, op);                        % ss, w_ref -> 17 sorties
assert(isequal(size(G),[17 4]), ...
    'G est %dx%d au lieu de 17x4 : verifier les Inports/Outports racine.', size(G,1), size(G,2));
fprintf('Ordre du modele linearise : %d etats\n', size(G.A,1));
G12 = G(1:12,:);                               % [xi v eta W] <- w_ref

%% --- (A,B) analytique et LQR (H7) ---
% NOTE 1 : adapter 'f' a la signature reelle de dyn (struct 'drone' ou vecteur params).
f  = @(x,u) dyn(x,u,drone);
x0 = zeros(12,1);  e = 1e-6;                   % 'e' evite de masquer eps
A = zeros(12,12);  B = zeros(12,4);
for i = 1:12
    xp = x0; xp(i) = e;  xm = x0; xm(i) = -e;
    A(:,i) = (f(xp,u_eq) - f(xm,u_eq))/(2*e);
end
for i = 1:4
    up = u_eq; up(i) = up(i)+e;  um = u_eq; um(i) = um(i)-e;
    B(:,i) = (f(x0,up) - f(x0,um))/(2*e);
end
d2r   = pi/180;
x_max = [1;1;0.5;2;2;2;15*d2r;15*d2r;45*d2r;1;1;1];
u_max = repmat(0.2*w_h,4,1);
Q = diag(1./x_max.^2);  R = diag(1./u_max.^2);
K = lqr(A,B,Q,R);                              % [4x12]

save('lqr_hover.mat','K','w_h');     % K [4x12], w_h [1x1] rad/s

G_an = ss(A,B,eye(12),zeros(12,4));            % modele de synthese

%% --- T3-a : stabilite en boucle fermee sur le modele complet ---
Tcl = feedback(G12, K);                        % retour negatif : du = -K dy
p   = pole(Tcl);
[~,ix] = sort(real(p),'descend');  p = p(ix);
fprintf('\nPoles les moins amortis (partie reelle decroissante) :\n');
disp(p(1:min(8,numel(p))).');
pf = p(abs(p) > 1e-3);                         % exclut le mode SoC (heuristique)
fprintf('T3-a : max Re(lambda), |lambda|>1e-3 = %.3e  (< 0 requis)\n', max(real(pf)));
fprintf('       pole minimal en |lambda|      = %.3e (mode SoC attendu ~ 0)\n', min(abs(p)));
assert(max(real(pf)) < 0, 'ECHEC T3-a : LQR instable sur le modele complet');

%% --- T3-b : retour de difference a l'entree de la planta ---
w  = logspace(-2,3,600);                       % [rad/s]
s_full = min(sigma(eye(4) + K*G12 , w), [], 1);
s_an   = min(sigma(eye(4) + K*G_an, w), [], 1);
fprintf('T3-b : min sigma(I+L) modele de synthese = %.3f (attendu >= 1)\n', min(s_an));
fprintf('       min sigma(I+L) modele complet     = %.3f\n', min(s_full));
figure; semilogx(w,s_an,'b-',w,s_full,'r-','LineWidth',1.3); grid on;
yline(1,'k--'); xlabel('\omega [rad/s]'); ylabel('\sigma_{min}(I+L_{in})');
legend('Synthese (dyn.m)','Modele complet','Location','southeast');

%% --- T3-c : ecart entre modeles ---
for wk = [0.5 2 10]
    Gf = freqresp(G12 ,wk);  Ga = freqresp(G_an,wk);
    fprintf('T3-c : w = %5.1f rad/s, erreur relative = %.2e\n', wk, norm(Gf-Ga)/norm(Ga));
end