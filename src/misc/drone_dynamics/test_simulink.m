% Définition du signal d'entrée
t = (0:0.01:10)';
u = drone.w_h*(1+0.5*sin(t));

% Création de l'objet timeseries
u_ts = timeseries(u, t);

% Simulation du modèle
simOut = sim('drone_dyn', ...
    'ExternalInput', 'u_ts', ...
    'StopTime', '10');