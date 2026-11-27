% Main entry point
% This script runs everything from the start
% For a simple plot of the solution and avoid solving the nlp again,
% please run plot_nlp_2.m
init_params
init_scenario
gen_hess_loc
nlp_solv_fmincon_2
plot_nlp_2