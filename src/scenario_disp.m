%% visu_env.m
%  Visualisation 3D de l'environnement de navigation — Quadrirotor
%  Adapté depuis visu_bateau_2D.m
%  Dépendances : init_params.m, scenario_fixed.m

clear; clc; close all;

load("data/params.mat")
load("data/scenario.mat")


n_buildings = scen.n_buildings;
buildings = scen.buildings;
x_min = scen.x_min;
y_min = scen.y_min;
z_min = scen.z_min;
x_max = scen.x_max;
y_max = scen.y_max;
z_max = scen.z_max;
ecart = scen.ecart;

start_pos = scen.start_pos;
target_pos = scen.target_pos;

start_vel = scen.start_vel;
target_vel = scen.target_vel;

%% --- Figure ---
figure; hold on; grid on; axis equal;
view(35, 25);                        % angle de vue 3D (azimut, élévation)
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('Zone de navigation — Quadrirotor 3D');
xlim([x_min, x_max]); ylim([y_min, y_max]); zlim([z_min, z_max]);

% Éclairage pour améliorer la lisibilité 3D
camlight('headlight'); lighting gouraud;

%% --- Bâtiments (pavés droits AABB) ---
% Couleur de base et couleur avec marge de sécurité
col_building = [0.6, 0.2, 0.2];     % rouge sombre  — volume solide
col_margin   = [0.9, 0.5, 0.3];     % orange clair  — marge de sécurité

for i = 1:n_buildings

    xc = buildings(i,1);
    yc = buildings(i,2);
    dx = buildings(i,3);
    dy = buildings(i,4);
    hz = buildings(i,5);

    % -- Volume solide du bâtiment --
    draw_box(xc, yc, dx, dy, hz, col_building, 0.75);

    % -- Marge de sécurité (volume fantôme) --
    draw_box(xc, yc, dx+ecart, dy+ecart, hz+ecart, col_margin, 0.12);

    % -- Étiquette --
    text(xc, yc, hz + 1.0, sprintf('B%d', i), ...
        'HorizontalAlignment', 'center', ...
        'FontSize', 9, 'FontWeight', 'bold', 'Color', 'k');
end

%% --- Sol (plan z = 0) ---
patch([x_min x_max x_max x_min], ...
      [y_min y_min y_max y_max], ...
      [0     0     0     0    ], ...
      [0.85 0.85 0.85], ...
      'FaceAlpha', 0.3, 'EdgeColor', 'none');

%% --- Départ ---
plot3(start_pos(1), start_pos(2), start_pos(3), 'gs', ...
    'MarkerSize', 14, 'MarkerFaceColor', 'g', ...
    'DisplayName', 'Départ');

%% --- Cible ---
plot3(target_pos(1), target_pos(2), target_pos(3), 'p', ...
    'MarkerSize', 18, 'MarkerFaceColor', 'y', ...
    'MarkerEdgeColor', 'k', 'DisplayName', 'Cible');

%% --- Ligne directe départ → cible (référence visuelle) ---
plot3([start_pos(1), target_pos(1)], ...
      [start_pos(2), target_pos(2)], ...
      [start_pos(3), target_pos(3)], ...
      'b--', 'LineWidth', 1.2, 'DisplayName', 'Ligne directe');

legend('Location', 'northwest');

%% =========================================================
%  Fonction locale : dessine un pavé droit centré en (xc,yc)
%  de demi-dimensions (dx, dy) et de hauteur hz
%  =========================================================
function draw_box(xc, yc, dx, dy, hz, col, alpha)
    % Coordonnées des 8 sommets
    % Numérotation :
    %   1-4 : face basse (z=0),  sens anti-horaire vu du dessus
    %   5-8 : face haute (z=hz), même ordre
    vx = xc + dx * [-1  1  1 -1  -1  1  1 -1];
    vy = yc + dy * [-1 -1  1  1  -1 -1  1  1];
    vz =            [0  0  0  0   hz hz hz hz];

    % Définition des 6 faces (indices dans vx/vy/vz)
    faces = [
        1 2 3 4;   % bas
        5 6 7 8;   % haut
        1 2 6 5;   % face avant  (y = yc-dy)
        3 4 8 7;   % face arrière (y = yc+dy)
        2 3 7 6;   % face droite  (x = xc+dx)
        1 4 8 5;   % face gauche  (x = xc-dx)
    ];

    patch('Vertices', [vx; vy; vz]', ...
          'Faces',    faces, ...
          'FaceColor', col, ...
          'FaceAlpha', alpha, ...
          'EdgeColor', 'k', ...
          'LineWidth', 0.8);
end