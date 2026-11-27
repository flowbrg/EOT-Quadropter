clear; clc; close all;

%% Settings

USE_CASADI = true; % CasADi (requires CasADi library), else fmincon
OBSTACLES  = true;  % Trajectory with obstacles (in init_scenario.m), else 
                    % simple trajectory
FORCE_RUN_SOLV = false; % Force rerun the solv script

%% Main

addpath("./model");

assert(isfile('data/params.mat'), 'Run init_params.m');
assert(isfile('data/scenario.mat'), 'Run init_scenario.m');

load('data/params.mat');
load('data/scenario.mat');

assert(~USE_CASADI || ~isempty(which('casadi.SX')), 'CasADi not found in path.');

if USE_CASADI, solver = 'casadi'; else, solver = 'fmincon'; end
if OBSTACLES,  suffix = '_obs';   else, suffix = '';        end

solv_script = ['solv_' solver suffix '.m'];
plot_script = ['plot_sol' suffix '.m'];
sol_file    = fullfile('sols', ['sol_' solver suffix '.mat']);
fprintf('Library : %s | obstacles : %d\n', solver, OBSTACLES);

if ~isfile(sol_file)
    fprintf('No solution file found: %s\n', solv_script);
    run(solv_script);
elseif hasChanged(sol_file)
    fprintf('Parameters are not up to date: running %s\n', solv_script);
    run(solv_script);
elseif FORCE_RUN_SOLV
    run(solv_script);
else
    fprintf('Solution file found and up to date: %s\n', sol_file);
    load(sol_file)
end

run(plot_script);

function changed = hasChanged(sol_file)
P = load('data/params.mat', 'drone', 'mot', 'batt');
S = load(sol_file, 'drone', 'mot', 'batt');
changed = false;
for f = {'drone', 'mot', 'batt'}
    if ~isfield(S, f{1}) || ~isequal(P.(f{1}), S.(f{1}))
        changed = true;
        return
    end
end
end