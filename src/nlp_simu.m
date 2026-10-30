%% Calcul de trajectoire ideale a priori
%  CasADi + IPOPT

clear; clc; close all;

nlp_solv

%% Simulation de la solution sur le modèle "réel" (sans collocation)
% Commandes idéales : maintien d'ordre zéro (constantes par intervalle),
% intégration ode45 intervalle par intervalle (pas de discontinuité
% à l'intérieur d'un appel à ode45).
options_real = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);

x_c    = X_sol(:,1);            % même état initial que le NLP
t_real = t_sol(1);
x_real = x_c.';

for k = 1:N
    u_k = U_sol(:,k)*w_h;       % [rad/s] 4x1
    [t_k, x_k] = ode45(@(t,x) dyn(x, u_k, p), ...
                       [t_sol(k) t_sol(k+1)], x_c, options_real);
    t_real = [t_real; t_k(2:end)];       %#ok<AGROW>
    x_real = [x_real; x_k(2:end,:)];     %#ok<AGROW>
    x_c    = x_k(end,:).';
end

% Erreurs terminales (réel vs cible)
fprintf('Erreur position finale : %.3f m\n', norm(x_real(end,1:3) - s.target_pos));
fprintf('Erreur vitesse finale  : %.3f m/s\n', norm(x_real(end,4:6) - s.target_vel));
fprintf('Ecart max idéal/réel (position) à Tf : %.3f m\n', ...
        norm(x_real(end,1:3) - X_sol(1:3,end).'));

%% Figure 1 : Trajectoires idéale et réelle

figure(1); clf;

% --- Vue 3D ---
subplot(1,2,1); hold on; grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title(sprintf('Trajectoire 3D  (Tf = %.1f s)', Tf_sol));
view(35, 25);
xlim([s.x_min s.x_max]); ylim([s.y_min s.y_max]); zlim([s.z_min s.z_max]);

plot3(s.start_pos(1), s.start_pos(2), s.start_pos(3), 'gs', ...
      'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', 'Départ');
plot3(s.target_pos(1), s.target_pos(2), s.target_pos(3), 'p', ...
      'MarkerSize', 16, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', ...
      'DisplayName', 'Cible');
plot3(X_sol(1,:), X_sol(2,:), X_sol(3,:), 'b-', 'LineWidth', 2, ...
      'DisplayName', 'Trajectoire idéale (collocation)');
plot3(x_real(:,1), x_real(:,2), x_real(:,3), 'r-', 'LineWidth', 2, ...
      'DisplayName', 'Trajectoire réelle');
legend('Location','northwest');

% --- Vue de dessus (xy) ---
subplot(1,2,2); hold on; grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)'); title('Vue de dessus');
xlim([s.x_min-1 s.x_max+1]); ylim([s.y_min-1 s.y_max+1]);

plot(s.start_pos(1), s.start_pos(2), 'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot(s.target_pos(1), s.target_pos(2), 'p', 'MarkerSize', 16, ...
     'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k');
plot(X_sol(1,:), X_sol(2,:), 'b-', 'LineWidth', 2);
plot(x_real(:,1), x_real(:,2), 'r-', 'LineWidth', 2);

% Direction de la vitesse (horizontale) - trajectoire idéale
n_arr = 15;
idx   = round(linspace(1, N+1, n_arr));
V_sol = max(hypot(X_sol(4,idx), X_sol(5,idx)), 1e-6);
quiver(X_sol(1,idx), X_sol(2,idx), ...
       2*X_sol(4,idx)./V_sol, 2*X_sol(5,idx)./V_sol, 0, 'k', ...
       'HandleVisibility', 'off');

% Direction de la vitesse (horizontale) - trajectoire réelle
idx_real = round(linspace(1, length(t_real), n_arr));
V_real   = max(hypot(x_real(idx_real,4), x_real(idx_real,5)), 1e-6);
quiver(x_real(idx_real,1), x_real(idx_real,2), ...
       2*x_real(idx_real,4)./V_real, 2*x_real(idx_real,5)./V_real, 0, ...
       'Color', [0.8500 0.3250 0.0980], 'HandleVisibility', 'off');

%% --- Figure 2 : Etats idéale vs réels ---
% Trait plein = idéal (collocation) ; trait pointillé = réel (ode45)

figure('Name', 'États');
col = lines(3);

% Position
subplot(4,1,1); hold on; grid on;
for i = 1:3
    plot(t_sol,  X_sol(i,:),  '-',  'Color', col(i,:), 'LineWidth', 1.5);
    plot(t_real, x_real(:,i), '--', 'Color', col(i,:), 'LineWidth', 1.5);
end
ylabel('Position (m)'); title('Position  (x, y, z)');
legend('x idéal','x réel','y idéal','y réel','z idéal','z réel', ...
       'Location','best', 'NumColumns', 3);

% Vitesse
subplot(4,1,2); hold on; grid on;
for i = 1:3
    plot(t_sol,  X_sol(3+i,:),  '-',  'Color', col(i,:), 'LineWidth', 1.5);
    plot(t_real, x_real(:,3+i), '--', 'Color', col(i,:), 'LineWidth', 1.5);
end
ylabel('v (m/s)'); title('Vitesse  (v_x, v_y, v_z)');

% Attitude
subplot(4,1,3); hold on; grid on;
for i = 1:3
    plot(t_sol,  rad2deg(X_sol(6+i,:)),  '-',  'Color', col(i,:), 'LineWidth', 1.5);
    plot(t_real, rad2deg(x_real(:,6+i)), '--', 'Color', col(i,:), 'LineWidth', 1.5);
end
ylabel('Angles (°)'); title('Attitude  (\phi, \theta, \psi)');

% Vitesses angulaires
subplot(4,1,4); hold on; grid on;
for i = 1:3
    plot(t_sol,  X_sol(9+i,:),  '-',  'Color', col(i,:), 'LineWidth', 1.5);
    plot(t_real, x_real(:,9+i), '--', 'Color', col(i,:), 'LineWidth', 1.5);
end
ylabel('\omega (rad/s)'); title('Vitesses angulaires  (p, q, r)');
xlabel('t (s)');

%% Figure 3 : Commandes et énergie
figure(3); clf;

t_u = t_sol(1:end-1);
Om  = U_sol*w_h;                             % [rad/s]

subplot(3,1,1); hold on; grid on;
stairs(t_u, Om(1,:), 'LineWidth', 1.5);
stairs(t_u, Om(2,:), 'LineWidth', 1.5);
stairs(t_u, Om(3,:), 'LineWidth', 1.5);
stairs(t_u, Om(4,:), 'LineWidth', 1.5);
yline(Omega_max, 'k--'); yline(Omega_min, 'k--'); yline(w_h, 'g:');
ylabel('\Omega_i (rad/s)'); title('Vitesses de rotation des hélices');
legend('\Omega_1','\Omega_2','\Omega_3','\Omega_4','Location','best', ...
       'NumColumns', 4);

subplot(3,1,2); grid on;
stairs(t_u, P_sol, 'm', 'LineWidth', 1.5); hold on;
yline(P_h, 'g:', 'hover');
ylabel('P (W)'); title('Puissance mécanique d''hélice  k_Q \Sigma\Omega_i^3');

subplot(3,1,3); grid on;
E_cum = [0, cumsum(P_sol .* diff(t_sol))];
plot(t_sol, E_cum, 'k', 'LineWidth', 1.5); hold on;
plot(t_sol, P_h*t_sol, 'g:', 'LineWidth', 1.2);
ylabel('E (J)'); xlabel('t (s)');
title(sprintf('Énergie cumulée  (E_f = %.1f J)', E_cum(end)));
legend('Trajectoire optimale', 'Hover seul', 'Location', 'northwest');