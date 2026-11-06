%% scenario_fixed.m
%  Scénario déterministe — un drone quadrirotor, une cible, 5 bâtiments
%  Obstacles : pavés droits axis-aligned (AABB)
%  Convention : chaque bâtiment défini par [xc, yc, dx, dy, hz]
%               avec (xc,yc) = centre du bâtiment au sol
%               dx, dy = demi-dimensions en x et y
%               hz = hauteur totale (le bâtiment occupe z in [0, hz])

scen = struct();

%% --- Départ et arrivée ---
scen.start_pos  = [0,  0,  0];   % [m] position initiale  (x, y, z)
scen.target_pos = [50, 50, 0];   % [m] position finale    (x, y, z)

scen.start_vel  = [0, 0, 0];     % [m/s] vitesse initiale nulle
scen.target_vel = [0, 0, 0];     % [m/s] vitesse finale nulle (atterrissage posé)

%% --- Bâtiments (obstacles AABB) ---
%  Format : [xc, yc, dx, dy, hz]
%  xc, yc   : centre au sol [m]
%  dx, dy   : demi-dimensions horizontales [m]  -> occupe [xc-dx, xc+dx] x [yc-dy, yc+dy]
%  hz       : hauteur totale [m]               -> occupe z in [0, hz]

scen.buildings = [
    10, 10,  3, 4, 12;   % B1 — fin et haut,    bloque la diagonale basse
    30, 15, 5, 3,  7;   % B2 — large,           centre gauche
    27, 40, 4, 4, 15;   % B3 — carré très haut, obstacle central (hz max)
    35, 25, 3, 6,  9;   % B4 — allongé en y,    zone médiane haute
    10, 25, 5, 5, 11;   % B5 — grand carré,     approche finale
    ];

scen.n_buildings = size(scen.buildings, 1);

%  Marge de sécurité autour des bâtiments [m]
%  À additionner à dx, dy, hz lors de la formulation des contraintes
scen.ecart = 1.0;    % 1 m de marge — cohérent avec l'envergure ~0.5 m du drone

%% --- Bornes de l'espace de vol ---
scen.x_min = 0;   scen.x_max = 55;   % [m]
scen.y_min = 0;   scen.y_max = 55;   % [m]
scen.z_min = 0;   scen.z_max = 30;   % [m] — altitude max opérationnelle

save("scenario.mat", "scen");