%% Calcul de trajectoire ideale a priori
% Trajectoire sans obstacle
% CasADi + IPOPT

clear; clc; close all;

nlp_solv
%nlp_alt

%% Simulation sur le modele complet (21 etats)

options_real = odeset('RelTol',1e-8,'AbsTol',1e-10);

% Optimised rotor speeds: physical rad/s
% IMPORTANT: U_sol now contains angular accelerations, not rotor speeds.
W_ref = X_sol(13:16,:);
%W_ref = U_sol(1:4,:);

% Initial battery state
SoC0 = 1.0;               % Same initial SoC as previous simulation
z0   = zeros(4,1);        % Controller integrator states

% Full model state:
% [mechanical states(12); rotor speeds(4); SoC; controller states(4)]
x_c = [X_sol(:,1); SoC0; z0];
%x_c = [X_sol(:,1);w_h*ones(4,1); SoC0; z0];

assert(numel(x_c)==21,'Initial state must contain 21 elements.');

t_real = t_sol(1);
x_real = x_c.';

for k = 1:N
    u_k = W_ref(:,k);       % [rad/s] 4x1
    [t_k, x_k] = ode45(@(t,x) dyn_uav(x, u_k, drone, batt, mot), ...
                       [t_sol(k) t_sol(k+1)], x_c, options_real);
    t_real = [t_real; t_k(2:end)];       %#ok<AGROW>
    x_real = [x_real; x_k(2:end,:)];     %#ok<AGROW>
    x_c    = x_k(end,:).';
end

%% Terminal errors

err_pos = norm(x_real(end,1:3).' - s.target_pos(:));
err_vel = norm(x_real(end,4:6).' - s.target_vel(:));

err_ideal_real = norm( ...
    x_real(end,1:3).' - X_sol(1:3,end));

fprintf('Erreur position finale : %.3f m\n',err_pos);
fprintf('Erreur vitesse finale  : %.3f m/s\n',err_vel);
fprintf('Ecart final ideal/reel : %.3f m\n',err_ideal_real);

fprintf('SoC initial : %.4f\n',x_real(1,17));
fprintf('SoC final   : %.4f\n',x_real(end,17));


%% ============================================================
% FIGURE 1 : Trajectoires ideale et reelle
% =============================================================

figure(1); clf;

% --- Vue 3D ---
subplot(1,2,1); hold on; grid on; axis equal;

xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');

title(sprintf('Trajectoire 3D (Tf = %.1f s)',Tf_sol));

view(35,25);

xlim([s.x_min s.x_max]);
ylim([s.y_min s.y_max]);
zlim([s.z_min s.z_max]);

plot3(s.start_pos(1),s.start_pos(2),s.start_pos(3), ...
    'gs','MarkerSize',12,'MarkerFaceColor','g', ...
    'DisplayName','Depart');

plot3(s.target_pos(1),s.target_pos(2),s.target_pos(3), ...
    'p','MarkerSize',16,'MarkerFaceColor','y', ...
    'MarkerEdgeColor','k','DisplayName','Cible');

plot3(X_sol(1,:),X_sol(2,:),X_sol(3,:), ...
    'b-','LineWidth',2, ...
    'DisplayName','Trajectoire ideale');

plot3(x_real(:,1),x_real(:,2),x_real(:,3), ...
    'r-','LineWidth',2, ...
    'DisplayName','Trajectoire reelle');

legend('Location','northwest');


% --- Vue de dessus ---
subplot(1,2,2); hold on; grid on; axis equal;

xlabel('x (m)');
ylabel('y (m)');
title('Vue de dessus');

xlim([s.x_min-1 s.x_max+1]);
ylim([s.y_min-1 s.y_max+1]);

plot(s.start_pos(1),s.start_pos(2), ...
    'gs','MarkerSize',12,'MarkerFaceColor','g');

plot(s.target_pos(1),s.target_pos(2), ...
    'p','MarkerSize',16,'MarkerFaceColor','y', ...
    'MarkerEdgeColor','k');

plot(X_sol(1,:),X_sol(2,:), ...
    'b-','LineWidth',2);

plot(x_real(:,1),x_real(:,2), ...
    'r-','LineWidth',2);


% Direction of the horizontal velocity - ideal trajectory
n_arr = 15;

idx = round(linspace(1,N+1,n_arr));

V_sol = max(hypot(X_sol(4,idx),X_sol(5,idx)),1e-6);

quiver(X_sol(1,idx),X_sol(2,idx), ...
    2*X_sol(4,idx)./V_sol, ...
    2*X_sol(5,idx)./V_sol, ...
    0,'k','HandleVisibility','off');


% Direction of horizontal velocity - real trajectory
idx_real = round(linspace(1,length(t_real),n_arr));

V_real = max(hypot( ...
    x_real(idx_real,4),x_real(idx_real,5)),1e-6);

quiver(x_real(idx_real,1),x_real(idx_real,2), ...
    2*x_real(idx_real,4)./V_real, ...
    2*x_real(idx_real,5)./V_real, ...
    0,'Color',[0.8500 0.3250 0.0980], ...
    'HandleVisibility','off');


%% ============================================================
% FIGURE 2 : Etats ideaux vs reels + SoC
% =============================================================

figure(2); clf;
set(gcf,'Name','Etats UAV et batterie', ...
    'Position',[100 50 1100 950]);

col = lines(3);


% --- 1. Position ---
subplot(5,1,1); hold on; grid on;

for i = 1:3

    plot(t_sol,X_sol(i,:), ...
        '-','Color',col(i,:),'LineWidth',1.5);

    plot(t_real,x_real(:,i), ...
        '--','Color',col(i,:),'LineWidth',1.5);

end

ylabel('Position (m)');
title('Position (x, y, z)');

legend('x ideal','x reel', ...
       'y ideal','y reel', ...
       'z ideal','z reel', ...
       'Location','best','NumColumns',3);


% --- 2. Velocity ---
subplot(5,1,2); hold on; grid on;

for i = 1:3

    plot(t_sol,X_sol(3+i,:), ...
        '-','Color',col(i,:),'LineWidth',1.5);

    plot(t_real,x_real(:,3+i), ...
        '--','Color',col(i,:),'LineWidth',1.5);

end

ylabel('v (m/s)');
title('Vitesse (v_x, v_y, v_z)');


% --- 3. Attitude ---
subplot(5,1,3); hold on; grid on;

for i = 1:3

    plot(t_sol,rad2deg(X_sol(6+i,:)), ...
        '-','Color',col(i,:),'LineWidth',1.5);

    plot(t_real,rad2deg(x_real(:,6+i)), ...
        '--','Color',col(i,:),'LineWidth',1.5);

end

ylabel('Angles (deg)');
title('Attitude (\phi, \theta, \psi)');

legend('\phi ideal','\phi reel', ...
       '\theta ideal','\theta reel', ...
       '\psi ideal','\psi reel', ...
       'Location','best','NumColumns',3);


% --- 4. Body angular velocities ---
subplot(5,1,4); hold on; grid on;

for i = 1:3

    plot(t_sol,X_sol(9+i,:), ...
        '-','Color',col(i,:),'LineWidth',1.5);

    plot(t_real,x_real(:,9+i), ...
        '--','Color',col(i,:),'LineWidth',1.5);

end

ylabel('\omega (rad/s)');
title('Vitesses angulaires (p, q, r)');


% --- 5. Battery state of charge ---
subplot(5,1,5); hold on; grid on;

plot(t_real,x_real(:,17), ...
    'k-','LineWidth',1.8);

yline(SoC0,'b:','SoC initial');

xlabel('Temps (s)');
ylabel('SoC (-)');

title('Etat de charge de la batterie');

%ylim([0 1]);

legend('SoC simule','Location','best');


%% ============================================================
% FIGURE 3 : Rotor speeds - reference vs actual

figure(3); clf;
set(gcf,'Name','Vitesses des moteurs', ...
    'Position',[100 100 1100 700]);

for i = 1:4

    subplot(2,2,i); hold on; grid on;

    % Optimized speed reference
    plot(t_sol,W_ref(i,:), ...
        'b-','LineWidth',1.8);

    % Actual rotor speed from complete simulation
    plot(t_real,x_real(:,12+i), ...
        'r-','LineWidth',1.5);

    % Rotor-speed bounds
    yline(w_max,'k--','HandleVisibility','off');
    yline(w_min,'k--','HandleVisibility','off');

    % Hover speed
    yline(w_h,'g:','HandleVisibility','off');

    ylabel(sprintf('\\omega_%d (rad/s)',i));
    xlabel('Temps (s)');

    title(sprintf('Moteur %d',i));

    if i==1
        legend('Reference optimisee', ...
               'Vitesse reelle', ...
               'Location','best');
    end

end

%% Figure 4: Position tracking error

% Time points of the optimized trajectory
t_nodes = linspace(t_real(1), t_real(end), size(X_sol, 2));

% Interpolate optimized position onto the ode45 time points
pos_expected = interp1(t_nodes, X_sol(1:3,:).', t_real, 'linear');

% Actual position from the full simulation
pos_real = x_real(:,1:3);

% Euclidean position error
pos_error = sqrt(sum((pos_real - pos_expected).^2, 2));

% Plot
figure(4);clf;
set(gcf,'Name','Erreur de suivi', ...
    'Position',[1000 70 700 550]);

plot(t_real, pos_error, 'LineWidth', 1.5);
grid on;

xlabel('Time [s]');
ylabel('Position error [m]');
title('Position error: real model vs optimized trajectory');