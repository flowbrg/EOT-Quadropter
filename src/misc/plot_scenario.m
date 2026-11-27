%% plot_scenario.m
%
%   MOVE TO SRC/
%
%  3D visualization of the navigation environment — Quadrotor
%  Adapted from visu_bateau_2D.m
%  Dependencies: init_params.m, scenario_fixed.m

clear; clc; close all;

load("./data/params.mat")
load("./data/scenario.mat")


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

%% Figure
figure; hold on; grid on; axis equal;
view(35, 25);                        % 3D viewing angle (azimuth, elevation)
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('Navigation Zone — 3D Quadrotor');
xlim([x_min, x_max]); ylim([y_min, y_max]); zlim([z_min, z_max]);

% Lighting to improve 3D readability
camlight('headlight'); lighting gouraud;

% Buildings (AABB boxes)
% Base color and safety margin color
col_building = [0.6, 0.2, 0.2];     % dark red — solid volume
col_margin   = [0.9, 0.5, 0.3];     % light orange — safety margin

for i = 1:n_buildings

    xc = buildings(i,1);
    yc = buildings(i,2);
    dx = buildings(i,3);
    dy = buildings(i,4);
    hz = buildings(i,5);

    % Solid building volume
    draw_box(xc, yc, dx, dy, hz, col_building, 0.75);

    % Safety margin (ghost volume)
    draw_box(xc, yc, dx+ecart, dy+ecart, hz+ecart, col_margin, 0.12);

    % Label
    text(xc, yc, hz + 1.0, sprintf('B%d', i), ...
        'HorizontalAlignment', 'center', ...
        'FontSize', 9, 'FontWeight', 'bold', 'Color', 'k');
end

% Ground (z = 0 plane)
patch([x_min x_max x_max x_min], ...
      [y_min y_min y_max y_max], ...
      [0     0     0     0    ], ...
      [0.85 0.85 0.85], ...
      'FaceAlpha', 0.3, 'EdgeColor', 'none');

% Start
plot3(start_pos(1), start_pos(2), start_pos(3), 'gs', ...
    'MarkerSize', 14, 'MarkerFaceColor', 'g', ...
    'DisplayName', 'Start');

% Target
plot3(target_pos(1), target_pos(2), target_pos(3), 'p', ...
    'MarkerSize', 18, 'MarkerFaceColor', 'y', ...
    'MarkerEdgeColor', 'k', 'DisplayName', 'Target');

legend('Location', 'northwest');

%%  Local function
% Draws an axis-aligned box centered at (xc,yc)
% with half-dimensions (dx, dy) and height hz
function draw_box(xc, yc, dx, dy, hz, col, alpha)
    % Coordinates of the 8 vertices
    % Numbering:
    %   1-4 : bottom face (z=0),  counter-clockwise viewed from above
    %   5-8 : top face (z=hz), same order
    vx = xc + dx * [-1  1  1 -1  -1  1  1 -1];
    vy = yc + dy * [-1 -1  1  1  -1 -1  1  1];
    vz =            [0  0  0  0   hz hz hz hz];

    % Definition of the 6 faces (indices into vx/vy/vz)
    faces = [
        1 2 3 4;   % bottom
        5 6 7 8;   % top
        1 2 6 5;   % front face  (y = yc-dy)
        3 4 8 7;   % back face   (y = yc+dy)
        2 3 7 6;   % right face  (x = xc+dx)
        1 4 8 5;   % left face   (x = xc-dx)
    ];

    patch('Vertices', [vx; vy; vz]', ...
          'Faces',    faces, ...
          'FaceColor', col, ...
          'FaceAlpha', alpha, ...
          'EdgeColor', 'k', ...
          'LineWidth', 0.8);
end