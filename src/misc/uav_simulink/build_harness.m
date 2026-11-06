function build_harness(mdl, harness)
% BUILD_HARNESS  Cree le modele 'harness' : LQR bouclant le modele 'mdl'.
% Variables du workspace de base lues par le harnais : K [4x12], u_eq [4x1].
% Prerequis : 'mdl' sauvegarde et dans le path ; 1 Inport [4], 6 Outports.

if bdIsLoaded(harness), close_system(harness, 0); end
new_system(harness);

add_block('simulink/Ports & Subsystems/Model', [harness '/Plant'], 'ModelName', mdl);
add_block('simulink/Signal Routing/Mux',       [harness '/Mux12'], 'Inputs', '4');
add_block('simulink/Math Operations/Gain',     [harness '/K'], ...
    'Gain', 'K', 'Multiplication', 'Matrix(K*u)');
add_block('simulink/Math Operations/Sum',      [harness '/Sum'], 'Inputs', '+-');
add_block('simulink/Sources/Constant',         [harness '/ueq'], 'Value', 'u_eq');
add_block('simulink/Discontinuities/Saturation', [harness '/Sat'], ...
    'LowerLimit', '0', 'UpperLimit', 'inf');

% Reshape vers 1-D (les Outports peuvent etre [n x 1] ou 1-D) puis Mux vers y12 [12]
for k = 1:4
    r = sprintf('Rsh%d', k);
    add_block('simulink/Math Operations/Reshape', [harness '/' r], ...
        'OutputDimensionality', '1-D array');
    add_line(harness, sprintf('Plant/%d', k), [r '/1']);
    add_line(harness, [r '/1'], sprintf('Mux12/%d', k));
end
add_line(harness, 'Mux12/1', 'K/1');
add_line(harness, 'ueq/1',   'Sum/1');
add_line(harness, 'K/1',     'Sum/2');
add_line(harness, 'Sum/1',   'Sat/1');
add_line(harness, 'Sat/1',   'Plant/1');

% Outports : 6 sorties du plant (xi, v, eta, W, w_i, SoC) + commande appliquee
for k = 1:6
    add_block('simulink/Sinks/Out1', sprintf('%s/y%d', harness, k), 'Port', num2str(k));
    add_line(harness, sprintf('Plant/%d', k), sprintf('y%d/1', k));
end
add_block('simulink/Sinks/Out1', [harness '/u_cmd'], 'Port', '7');
add_line(harness, 'Sat/1', 'u_cmd/1');

save_system(harness);
end