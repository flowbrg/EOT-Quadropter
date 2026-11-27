%% Simulation of the ideal a priori trajectory
% Trajectory with obstacles
% Simulation: full 21-state model

clear; clc; close all;

% UNCOMMENT THE SOLUTION TO DISPLAY
% Optimization: MATLAB fmincon (nlp_solv_fmincon_2.m)
load("./sols/sol_fmincon_2.mat");  % X_sol, U_sol, t_sol, Tf_sol
% Optimization: CasADi + IPOPT (old/nlp_solv_casadi_2.m)
%load("./sols/sol_casadi_2.mat");    % X_sol, U_sol, t_sol, Tf_sol
load("./data/params.mat");          % Must define drone, mot, and batt
load("./data/scenario.mat");        % Defines scen
addpath("./model");

% Number of intervals
N = size(X_sol,2)-1;

% Motor parameters
w_h   = drone.w_h;
w_min = 0.1*w_h;
w_max = 1.8*w_h;

% Dimension checks
assert(size(X_sol,1)==16, 'X_sol must contain 16 states.');
assert(size(U_sol,1)==4,  'U_sol must contain 4 inputs.');
assert(size(X_sol,2)==numel(t_sol), 'Incompatible dimensions between X_sol and t_sol.');
assert(exist('batt','var')==1, 'Variable batt is missing. Load battery parameters.');

t_sol = t_sol(:);

fprintf('Final time         : %.3f s\n',Tf_sol);
fprintf('Number of intervals: %d\n',N);
if exist('E_sol','var')
    fprintf('NLP energy         : %.1f J\n',E_sol);
end

%% Simulation on the full model (21 states)

options_real = odeset('RelTol',1e-8,'AbsTol',1e-10);

% X_sol(13:16,:) : rotor speeds [rad/s] (reference)
% U_sol          : angular accelerations [rad/s^2]
W_ref = X_sol(13:16,:);

SoC0 = 1.0;
z0   = zeros(4,1);

% Full state: [mech states (12); rotors (4); SoC; controller states (4)]
x_c = [X_sol(:,1); SoC0; z0];
assert(numel(x_c)==21, 'Initial state must contain 21 elements.');

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

fprintf('\n--- Terminal errors ---\n');
fprintf('Final position error : %.5f m\n',err_pos);
fprintf('Final velocity error : %.5f m/s\n',err_vel);
fprintf('Attitude error       : %.5f rad\n',err_att);
fprintf('Final ideal/real diff: %.5f m\n',err_ideal_real);

fprintf('\n--- Battery ---\n');
fprintf('Initial SoC : %.5f\n',x_real(1,17));
fprintf('Final SoC   : %.5f\n',x_real(end,17));

%% Exact signed distance to AABB boxes
% sd > 0 : outside ; sd < 0 : inside. Required margin: sd >= scen.ecart.

sd_min = @(P) min(cell2mat(arrayfun(@(b) signed_dist_aabb(P, scen.buildings(b,:)), ...
    1:scen.n_buildings, 'UniformOutput', false)), [], 2);

sd_real = sd_min(x_real(:,1:3));
sd_nlp  = sd_min(X_sol(1:3,:).');

fprintf('\n--- Obstacles ---\n');
fprintf('Min building clearance : actual (ode45) = %.3f m | NLP nodes = %.3f m | required = %.2f m\n', ...
    min(sd_real), min(sd_nlp), scen.ecart);
if min(sd_real) < scen.ecart
    warning('Safety margin not respected on the actual trajectory (corners/edges or between nodes).');
end

%% Figure 1: Ideal and actual trajectories

figure(1); clf;
set(gcf,'Name','fmincon Trajectories (obstacles)');

% --- 3D View ---
subplot(1,2,1); hold on; grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title(sprintf('3D Trajectory (Tf = %.1f s)',Tf_sol));
view(35,25);

% Buildings (box [xc-dx,xc+dx] x [yc-dy,yc+dy] x [0,hz])
Vc = [-1 -1 0; 1 -1 0; 1 1 0; -1 1 0; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1];
Fc = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
for b = 1:scen.n_buildings
    bd = scen.buildings(b,:);                       % [xc yc dx dy hz]
    Vb = Vc.*[bd(3) bd(4) bd(5)] + [bd(1) bd(2) 0];
    patch('Vertices',Vb,'Faces',Fc,'FaceColor',[0.8 0.2 0.2], ...
        'FaceAlpha',0.5,'EdgeColor','k','HandleVisibility','off');
end

xlim([scen.x_min scen.x_max]);
ylim([scen.y_min scen.y_max]);
zlim([scen.z_min scen.z_max]);

plot3(scen.start_pos(1),scen.start_pos(2),scen.start_pos(3), ...
    'gs','MarkerSize',12,'MarkerFaceColor','g','DisplayName','Start');
plot3(scen.target_pos(1),scen.target_pos(2),scen.target_pos(3), ...
    'p','MarkerSize',16,'MarkerFaceColor','y', ...
    'MarkerEdgeColor','k','DisplayName','Target');
plot3(X_sol(1,:),X_sol(2,:),X_sol(3,:), ...
    'b-','LineWidth',2,'DisplayName','Ideal fmincon trajectory');
plot3(x_real(:,1),x_real(:,2),x_real(:,3), ...
    'r-','LineWidth',2,'DisplayName','Actual trajectory');

legend('Location','northwest');

% --- Top View ---
subplot(1,2,2); hold on; grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)');
title('Top View');
xlim([scen.x_min-1 scen.x_max+1]);
ylim([scen.y_min-1 scen.y_max+1]);

for b = 1:scen.n_buildings
    bd = scen.buildings(b,:);
    rectangle('Position',[bd(1)-bd(3), bd(2)-bd(4), 2*bd(3), 2*bd(4)], ...
        'FaceColor',[0.8 0.2 0.2 0.5],'EdgeColor','k');
    text(bd(1),bd(2),sprintf('B%d (%.0f m)',b,bd(5)), ...
        'HorizontalAlignment','center','FontSize',8);
end

plot(scen.start_pos(1),scen.start_pos(2),'gs','MarkerSize',12,'MarkerFaceColor','g');
plot(scen.target_pos(1),scen.target_pos(2),'p','MarkerSize',16, ...
    'MarkerFaceColor','y','MarkerEdgeColor','k');
plot(X_sol(1,:),X_sol(2,:),'b-','LineWidth',2);
plot(x_real(:,1),x_real(:,2),'r-','LineWidth',2);

% Horizontal velocity direction - ideal trajectory
n_arr = 15;
idx = round(linspace(1,N+1,n_arr));
V_sol = max(hypot(X_sol(4,idx),X_sol(5,idx)),1e-6);
quiver(X_sol(1,idx),X_sol(2,idx), ...
    2*X_sol(4,idx)./V_sol, 2*X_sol(5,idx)./V_sol, ...
    0,'k','HandleVisibility','off');

% Horizontal velocity direction - actual trajectory
idx_real = round(linspace(1,length(t_real),n_arr));
V_real = max(hypot(x_real(idx_real,4),x_real(idx_real,5)),1e-6);
quiver(x_real(idx_real,1),x_real(idx_real,2), ...
    2*x_real(idx_real,4)./V_real, 2*x_real(idx_real,5)./V_real, ...
    0,'Color',[0.8500 0.3250 0.0980],'HandleVisibility','off');

legend('Start','Target', ...
    'Ideal trajectory','Actual trajectory', ...
    'Location','best');

%% Figure 2: Ideal vs actual states + SoC

figure(2); clf;
set(gcf,'Name','UAV and battery states - fmincon', ...
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
legend('x ideal','x actual','y ideal','y actual','z ideal','z actual', ...
       'Location','best','NumColumns',3);

% --- 2. Velocity ---
subplot(5,1,2); hold on; grid on;
for i = 1:3
    plot(t_sol,X_sol(3+i,:),'-','Color',col(i,:),'LineWidth',1.5);
    plot(t_real,x_real(:,3+i),'--','Color',col(i,:),'LineWidth',1.5);
end
ylabel('v (m/s)');
title('Velocity (v_x, v_y, v_z)');

% --- 3. Attitude ---
subplot(5,1,3); hold on; grid on;
for i = 1:3
    plot(t_sol,rad2deg(X_sol(6+i,:)),'-','Color',col(i,:),'LineWidth',1.5);
    plot(t_real,rad2deg(x_real(:,6+i)),'--','Color',col(i,:),'LineWidth',1.5);
end
ylabel('Angles (deg)');
title('Attitude (\phi, \theta, \psi)');
legend('\phi ideal','\phi actual','\theta ideal','\theta actual', ...
       '\psi ideal','\psi actual','Location','best','NumColumns',3);

% --- 4. Angular velocities ---
subplot(5,1,4); hold on; grid on;
for i = 1:3
    plot(t_sol,X_sol(9+i,:),'-','Color',col(i,:),'LineWidth',1.5);
    plot(t_real,x_real(:,9+i),'--','Color',col(i,:),'LineWidth',1.5);
end
ylabel('\omega (rad/s)');
title('Angular velocities (p, q, r)');

% --- 5. Battery state of charge ---
subplot(5,1,5); hold on; grid on;
plot(t_real,x_real(:,17),'k-','LineWidth',1.8);
yline(SoC0,'b:','Initial SoC');
xlabel('Time (s)');
ylabel('SoC (-)');
title('Battery state of charge');
legend('Simulated SoC','Location','best');

%% Figure 3: Rotor speeds - reference vs actual

figure(3); clf;
set(gcf,'Name','Motor speeds - fmincon', ...
    'Position',[100 100 1100 700]);

for i = 1:4
    subplot(2,2,i); hold on; grid on;

    plot(t_sol,W_ref(i,:),'b-','LineWidth',1.8);
    plot(t_real,x_real(:,12+i),'r-','LineWidth',1.5);

    %yline(w_max,'k--','HandleVisibility','off');
    %yline(w_min,'k--','HandleVisibility','off');
    yline(w_h,'g:','HandleVisibility','off');

    ylabel(sprintf('\\omega_%d (rad/s)',i));
    xlabel('Time (s)');
    title(sprintf('Motor %d',i));

    if i==1
        legend('Optimized reference','Actual speed','Location','best');
    end
end

%% Figure 4: Distance to closest building

pos_expected = interp1(t_sol,X_sol(1:3,:).',t_real,'linear');
pos_real = x_real(:,1:3);
pos_error = sqrt(sum((pos_real-pos_expected).^2,2));

figure(4); clf;
set(gcf,'Name','Tracking error - fmincon', ...
    'Position',[1000 70 700 700]);

subplot(2,1,1);
plot(t_real,pos_error,'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Position error (m)');
title('Tracking error: actual model vs fmincon trajectory');

subplot(2,1,2); hold on; grid on;
plot(t_real,sd_real,'r-','LineWidth',1.5);
plot(t_sol,sd_nlp,'b.','MarkerSize',8);
yline(scen.ecart,'k--','Required margin');
xlabel('Time (s)');
ylabel('Signed distance (m)');
title('Signed distance to closest building');
legend('Actual (ode45)','NLP nodes','Location','best');

%% Figure 5: Optimized inputs (Rotor accelerations)

figure(5); clf;
set(gcf,'Name','Angular accelerations - fmincon', ...
    'Position',[100 100 1100 700]);

for i = 1:4
    subplot(2,2,i); hold on; grid on;

    plot(t_sol,U_sol(i,:),'b-','LineWidth',1.8);

    ylabel(sprintf('d\\omega_{%d}/dt (rad/s^2)', i));
    xlabel('Time (s)');
    title(sprintf('Rotor %d input',i));
end

fprintf('\nfmincon simulation completed.\n');

%% Local function
function sd = signed_dist_aabb(P, bd)
% P : M x 3 positions ; bd = [xc yc dx dy hz] ; box occupying z in [0, hz]
% Output : M x 1 exact signed distance (box SDF convention)
q  = abs(P - [bd(1) bd(2) bd(5)/2]) - [bd(3) bd(4) bd(5)/2];
sd = vecnorm(max(q, 0), 2, 2) + min(max(q, [], 2), 0);
end