%% init_scenario.m
%  Deterministic scenario — one quadrotor drone, one target, 5 buildings (6 listed below)
%  Obstacles: axis-aligned bounding boxes (AABB)
%  Convention: each building is defined by [xc, yc, dx, dy, hz]
%              with (xc, yc) = building center on the ground
%              dx, dy = half-dimensions along x and y
%              hz = total height (the building occupies z in [0, hz])

scen = struct();

%% Start and arrival
scen.start_pos  = [0,  0,  0];   % [m] initial position  (x, y, z)
scen.target_pos = [50, 50, 0];   % [m] final position    (x, y, z)

scen.start_vel  = [0, 0, 0];     % [m/s] zero initial velocity
scen.target_vel = [0, 0, 0];     % [m/s] zero final velocity (touchdown landing)

%% Buildings (AABB obstacles)
%  Format: [xc, yc, dx, dy, hz]
%  xc, yc  : center on the ground [m]
%  dx, dy  : horizontal half-dimensions [m] -> occupies [xc-dx, xc+dx] x [yc-dy, yc+dy]
%  hz      : total height [m]              -> occupies z in [0, hz]

scen.buildings = [
    10, 10,  3, 4, 12;   % B1 — thin and tall,     blocks the lower diagonal
    30, 15,  5, 3,  7;   % B2 — wide,              center-left
    27, 40,  4, 4, 15;   % B3 — very tall square,  central obstacle (max hz)
    35, 25,  3, 6,  9;   % B4 — elongated in y,    upper middle zone
    10, 25,  5, 5, 11;   % B5 — large square,      final approach
    25, 25,  2, 2,  5;
    ];

scen.n_buildings = size(scen.buildings, 1);

%  Safety margin around buildings [m]
%  To be added to dx, dy, hz when formulating the constraints
scen.ecart = 1.0;    % 1 m margin — consistent with the ~0.5 m wingspan of the drone

%% Flight space bounds
scen.x_min = 0;   scen.x_max = 55;   % [m]
scen.y_min = 0;   scen.y_max = 55;   % [m]
scen.z_min = 0;   scen.z_max = 30;   % [m] — max operational altitude

save("data/scenario.mat", "scen");