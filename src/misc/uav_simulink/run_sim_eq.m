function [t, Y] = run_sim_eq(mdl, w_ref, t_final, xi0, w0_motor, SoC_0)
% RUN_SIM_EQ  Simule le modele Simulink avec une consigne w_ref constante.
%
% Entrees
%   mdl       char    nom du modele Simulink (ex. 'simu_aug')
%   w_ref     [4x1]   consigne de vitesse moteur          [rad/s]
%   t_final   [1x1]   duree de simulation                 [s]
%   xi0       [3x1]   position initiale                   [m]
%   w0_motor  [4x1]   vitesse moteur initiale (a lier aux IC du BLDCM)  [rad/s]
%   SoC_0     [1x1]   SoC initial dans [0,1] (a lier a l'IC du bloc Battery)
%
% Sorties
%   t  [Nx1]   instants du solveur (pas variable)         [s]
%   Y  [Nx17]  [xi(3) v(3) eta(3) W(3) w_i(4) SoC(1)]
%              (ordre des Outports 1..6, cf. hypothese H2)
%
% Prerequis : variables drone, battery, mot dans le workspace de base ;
%             les 6 Outports racine sont des signaux vecteurs 1-D.

assert(isequal(size(w_ref),[4 1]),    'w_ref doit etre [4x1]');
assert(isequal(numel(xi0),3),         'xi0 doit contenir 3 elements');
assert(isequal(size(w0_motor),[4 1]), 'w0_motor doit etre [4x1]');

load_system(mdl);
in = Simulink.SimulationInput(mdl);

% Parametres du solveur et format de sortie (Array : colonnes = Outports concatenes)
in = in.setModelParameter( ...
    'Solver',            'ode45', ...
    'RelTol',            '1e-8', ...
    'AbsTol',            '1e-10', ...
    'StopTime',          num2str(t_final,'%.10g'), ...
    'LoadExternalInput', 'on', ...
    'ExternalInput',     'u_ext', ...
    'SaveTime',          'on',  'TimeSaveName',   'tout', ...
    'SaveOutput',        'on',  'OutputSaveName', 'yout', ...
    'SaveFormat',        'Dataset');

% Entree externe au format structure : un seul Inport de dimension 4.
%   time              [2x1]  instants [s]
%   signals.values    [2x4]  une ligne par instant, une colonne par moteur [rad/s]
%   signals.dimensions = 4   dimension du signal de l'Inport (vecteur 4x1)
u_ext.time               = [0; t_final];
u_ext.signals.values     = [w_ref.'; w_ref.'];   % consigne constante
u_ext.signals.dimensions = 4;
in = in.setVariable('u_ext', u_ext);

% Conditions initiales (effectives seulement si liees dans le modele)
in = in.setVariable('xi0',      xi0(:));
in = in.setVariable('w0_motor', w0_motor);
in = in.setVariable('SoC_0',    SoC_0);

out = sim(in);
if ~isempty(out.ErrorMessage)
    error('run_sim_eq:simError', 'Erreur Simulink : %s', out.ErrorMessage);
end

% --- extraction des sorties (apres sim) ---
ys = out.yout;                       % Simulink.SimulationData.Dataset, 6 elements
assert(ys.numElements == 6, ...
    'yout contient %d elements au lieu de 6 Outports.', ys.numElements);

t = ys{1}.Values.Time;               % [Nx1] instants communs aux 6 signaux
nCol = [3 3 3 3 4 1];                % colonnes attendues par Outport (H2)
Y = zeros(numel(t), 17);
c = 0;
for k = 1:6
    d = ys{k}.Values.Data;           % [N x n], [N x n x 1] ou [n x 1 x N]
    if ndims(d) == 3 && size(d,1) == nCol(k) && size(d,3) == numel(t)
        d = permute(d, [3 1 2]);     % [n x 1 x N] -> [N x n]
    end
    d = reshape(d, numel(t), []);    % [N x n]
    assert(size(d,2) == nCol(k), ...
        'Outport %d : %d colonnes au lieu de %d.', k, size(d,2), nCol(k));
    Y(:, c+1:c+nCol(k)) = d;
    c = c + nCol(k);
end
end