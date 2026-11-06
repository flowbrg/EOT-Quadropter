
% Feedforward plot for a single BLDCM

clear; clc; close all;

load("params.mat");

N = 1000;

% Speed reference range
w_ref = linspace(0, 10*drone.w_h, N);

% Feedforward calculation
[Dc, valid] = ff(w_ref);

% Find maximum reachable speed (Dc <= 1)
idx_max = find(valid & Dc <= 1, 1, 'last');

if isempty(idx_max)
    error('No physically reachable operating point found.');
end

w_max = w_ref(idx_max);

% Plot only physically reachable region
figure(1);
plot(w_ref(valid & Dc <= 1), Dc(valid & Dc <= 1), ...
     'b-', 'LineWidth', 1.8);

hold on;
yline(1, 'r--', 'Maximum duty-cycle', 'LineWidth', 1.2);
xline(w_max, 'k--', ...
      sprintf('Maximum speed = %.1f rad/s', w_max), ...
      'LineWidth', 1.2);

grid on;
xlabel('Angular speed reference (rad/s)');
ylabel('Duty-cycle');
title('Feedforward characteristic');

xlim([0 w_max]);
ylim([0 1.1]);

fprintf('Maximum reachable speed: %.2f rad/s\n', w_max);
fprintf('Maximum reachable duty-cycle: %.4f\n', Dc(idx_max));