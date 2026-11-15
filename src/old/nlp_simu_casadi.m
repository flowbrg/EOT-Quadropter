%% Calcul de trajectoire ideale a priori
% Trajectoire sans obstacle
% CasADi + IPOPT

clear; clc; close all;

load("./sols/casadi_sol.mat");
load("data/params.mat");
load("data/scenario.mat");
addpath("./model");

% Nombre d'intervalles
N = size(X_sol,2)-1;

% Parametres moteurs
w_h = drone.w_h;
w_min = 0.1*w_h;
w_max = 1.8*w_h;

% Verification des dimensions
assert(size(X_sol,1)==16, 'X_sol doit contenir 16 etats.');
assert(size(U_sol,1)==4,  'U_sol doit contenir 4 commandes.');
assert(size(X_sol,2)==numel(t_sol), 'Dimensions incompatibles entre X_sol et t_sol.');
assert(exist('batt','var')==1, 'La variable batt est absente. Charger les parametres batterie.');

t_sol = t_sol(:);

fprintf('\n========================================\n');
fprintf('  SIMULATION TRAJECTOIRE CASADI\n');
fprintf('========================================\n');

fprintf('Temps final       : %.3f s\n',Tf_sol);
fprintf('Nombre intervalles: %d\n',N);
if exist('E_sol','var')
    fprintf('Energie NLP       : %.1f J\n',E_sol);
end

%% Simulation sur le modele complet (21 etats)

options_real = odeset('RelTol',1e-8,'AbsTol',1e-10);

% X_sol(13:16,:) : vitesses des rotors [rad/s] (consigne)
% U_sol          : accelerations angulaires [rad/s^2]
W_ref = X_sol(13:16,:);

SoC0 = 1.0;
z0   = zeros(4,1);

% Etat complet : [etats meca (12); rotors (4); SoC; etats controleur (4)]
x_c = [X_sol(:,1); SoC0; z0];
assert(numel(x_c)==21, 'L''etat initial doit contenir 21 elements.');

t_real = t_sol(1);
x_real = x_c.';

t_parts = cell(N,1);
x_parts = cell(N,1);

for k = 1:N
    u_k = W_ref(:,k);

    [t_k,x_k] = ode45( ...
        @(t,x) dyn_uav(x,u_k,drone,batt,mot), ...
        [t_sol(k) t_sol(k+1)], x_c, options_real);

    t_parts{k} = t_k(2:end);
    x_parts{k} = x_k(2:end,:);
    x_c = x_k(end,:).';
end

t_real = [t_real; vertcat(t_parts{:})];
x_real = [x_real; vertcat(x_parts{:})];

%% Terminal errors

err_pos = norm(x_real(end,1:3).' - scen.target_pos(:));
err_vel = norm(x_real(end,4:6).' - scen.target_vel(:));
err_ideal_real = norm(x_real(end,1:3).' - X_sol(1:3,end));
err_att = norm(x_real(end,7:8));

fprintf('\n--- Erreurs terminales ---\n');
fprintf('Erreur position finale : %.5f m\n',err_pos);
fprintf('Erreur vitesse finale  : %.5f m/s\n',err_vel);
fprintf('Erreur attitude        : %.5f rad\n',err_att);
fprintf('Ecart final ideal/reel : %.5f m\n',err_ideal_real);

fprintf('\n--- Batterie ---\n');
fprintf('SoC initial : %.5f\n',x_real(1,17));
fprintf('SoC final   : %.5f\n',x_real(end,17));


%% Figure 1 : Trajectoires ideale et reelle

figure(1); clf;
set(gcf,'Name','Trajectoires fmincon');

% --- Vue 3D ---
subplot(1,2,1); hold on; grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title(sprintf('Trajectoire 3D (Tf = %.1f s)',Tf_sol));
view(35,25);

xlim([scen.x_min scen.x_max]);
ylim([scen.y_min scen.y_max]);
zlim([scen.z_min scen.z_max]);

plot3(scen.start_pos(1),scen.start_pos(2),scen.start_pos(3), ...
    'gs','MarkerSize',12,'MarkerFaceColor','g', 'DisplayName','Depart');
plot3(scen.target_pos(1),scen.target_pos(2),scen.target_pos(3), ...
    'p','MarkerSize',16,'MarkerFaceColor','y', ...
    'MarkerEdgeColor','k','DisplayName','Cible');
plot3(X_sol(1,:),X_sol(2,:),X_sol(3,:), ...
    'b-','LineWidth',2, 'DisplayName','Trajectoire ideale fmincon');
plot3(x_real(:,1),x_real(:,2),x_real(:,3), ...
    'r-','LineWidth',2, 'DisplayName','Trajectoire reelle');

legend('Location','northwest');

% --- Vue de dessus ---
subplot(1,2,2); hold on; grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)');
title('Vue de dessus');

xlim([scen.x_min-1 scen.x_max+1]);
ylim([scen.y_min-1 scen.y_max+1]);

plot(scen.start_pos(1),scen.start_pos(2), 'gs','MarkerSize',12,'MarkerFaceColor','g');
plot(scen.target_pos(1),scen.target_pos(2), 'p','MarkerSize',16, ...
    'MarkerFaceColor','y', 'MarkerEdgeColor','k');
plot(X_sol(1,:),X_sol(2,:), 'b-','LineWidth',2);
plot(x_real(:,1),x_real(:,2), 'r-','LineWidth',2);

% Direction de la vitesse horizontale - trajectoire ideale
n_arr = 15;
idx = round(linspace(1,N+1,n_arr));
V_sol = max(hypot(X_sol(4,idx),X_sol(5,idx)),1e-6);
quiver(X_sol(1,idx),X_sol(2,idx), ...
    2*X_sol(4,idx)./V_sol, 2*X_sol(5,idx)./V_sol, ...
    0,'k','HandleVisibility','off');

% Direction de la vitesse horizontale - trajectoire reelle
idx_real = round(linspace(1,length(t_real),n_arr));
V_real = max(hypot(x_real(idx_real,4),x_real(idx_real,5)),1e-6);
quiver(x_real(idx_real,1),x_real(idx_real,2), ...
    2*x_real(idx_real,4)./V_real, 2*x_real(idx_real,5)./V_real, ...
    0,'Color',[0.8500 0.3250 0.0980], 'HandleVisibility','off');

legend('Depart','Cible', ...
    'Trajectoire ideale','Trajectoire reelle', ...
    'Location','best');


%% Figure 2 : Etats ideaux vs reels + SoC

figure(2); clf;
set(gcf,'Name','Etats UAV et batterie - fmincon', ...
    'Position',[100 50 1100 950]);

col = lines(3);

% --- 1. Position ---
subplot(5,1,1); hold on; grid on;
for i = 1:3
    plot(t_sol,X_sol(i,:),'-','Color',col(i,:),'LineWidth',1.5);
    plot(t_real,x_real(:,i),'--','Color',col(i,:),'LineWidth',1.5);
end
ylabel('Position (m)');
title('Position (x, y, z)');
legend('x ideal','x reel','y ideal','y reel','z ideal','z reel', ...
       'Location','best','NumColumns',3);

% --- 2. Vitesse ---
subplot(5,1,2); hold on; grid on;
for i = 1:3
    plot(t_sol,X_sol(3+i,:),'-','Color',col(i,:),'LineWidth',1.5);
    plot(t_real,x_real(:,3+i),'--','Color',col(i,:),'LineWidth',1.5);
end
ylabel('v (m/s)');
title('Vitesse (v_x, v_y, v_z)');

% --- 3. Attitude ---
subplot(5,1,3); hold on; grid on;
for i = 1:3
    plot(t_sol,rad2deg(X_sol(6+i,:)),'-','Color',col(i,:),'LineWidth',1.5);
    plot(t_real,rad2deg(x_real(:,6+i)),'--','Color',col(i,:),'LineWidth',1.5);
end
ylabel('Angles (deg)');
title('Attitude (\phi, \theta, \psi)');
legend('\phi ideal','\phi reel','\theta ideal','\theta reel', ...
       '\psi ideal','\psi reel','Location','best','NumColumns',3);

% --- 4. Vitesses angulaires ---
subplot(5,1,4); hold on; grid on;
for i = 1:3
    plot(t_sol,X_sol(9+i,:),'-','Color',col(i,:),'LineWidth',1.5);
    plot(t_real,x_real(:,9+i),'--','Color',col(i,:),'LineWidth',1.5);
end
ylabel('\omega (rad/s)');
title('Vitesses angulaires (p, q, r)');

% --- 5. Etat de charge batterie ---
subplot(5,1,5); hold on; grid on;
plot(t_real,x_real(:,17),'k-','LineWidth',1.8);
yline(SoC0,'b:','SoC initial');
xlabel('Temps (s)');
ylabel('SoC (-)');
title('Etat de charge de la batterie');
legend('SoC simule','Location','best');


%% Figure 3 : Rotor speeds - reference vs actual

figure(3); clf;
set(gcf,'Name','Vitesses des moteurs - fmincon', ...
    'Position',[100 100 1100 700]);

for i = 1:4
    subplot(2,2,i); hold on; grid on;

    plot(t_sol,W_ref(i,:),'b-','LineWidth',1.8);
    plot(t_real,x_real(:,12+i),'r-','LineWidth',1.5);

    %yline(w_max,'k--','HandleVisibility','off');
    %yline(w_min,'k--','HandleVisibility','off');
    yline(w_h,'g:','HandleVisibility','off');

    ylabel(sprintf('\\omega_%d (rad/s)',i));
    xlabel('Temps (s)');
    title(sprintf('Moteur %d',i));

    if i==1
        legend('Reference optimisee','Vitesse reelle','Location','best');
    end
end

%% Figure 4: Position tracking error

pos_expected = interp1(t_sol,X_sol(1:3,:).',t_real,'linear');
pos_real = x_real(:,1:3);
pos_error = sqrt(sum((pos_real-pos_expected).^2,2));

figure(4); clf;
set(gcf,'Name','Erreur de suivi - fmincon', ...
    'Position',[1000 70 700 700]);

set(gcf,'Name','Erreur de suivi - fmincon', ...
    'Position',[1000 70 700 550]);

plot(t_real,pos_error,'LineWidth',1.5);
grid on;
xlabel('Temps (s)');
ylabel('Erreur de position (m)');
title('Erreur de suivi : modele reel vs trajectoire fmincon');


%% Figure 5 : Commandes optimisees (Acceleration des rotors)

figure(5); clf;
set(gcf,'Name','Accelerations angulaires - fmincon', ...
    'Position',[100 100 1100 700]);

for i = 1:4
    subplot(2,2,i); hold on; grid on;

    plot(t_sol,U_sol(i,:),'b-','LineWidth',1.8);

    ylabel(sprintf('d\\omega_{%d}/dt (rad/s^2)', i));
    xlabel('Temps (s)');
    title(sprintf('Commande rotor %d',i));
end

fprintf('\nSimulation fmincon terminee.\n');