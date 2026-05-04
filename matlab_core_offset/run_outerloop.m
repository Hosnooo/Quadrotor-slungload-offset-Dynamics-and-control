clear; close all; clc;

addpath('simulationtools');

% =========================================================
% USER SETTINGS
% =========================================================

ref_fun = @ref_fig8;      % change this: @ref_fig8, @ref_spiral, @ref_helix, etc.
tf = 30.0;

save_dir = fullfile(pwd, 'results_offset_outerloop', func2str(ref_fun));

% =========================================================
% MODEL AND CONTROLLER
% =========================================================

model_fun = @model_QSFA_U;
controller_fun = @controller_QSFA_U;

% =========================================================
% PARAMETERS
% =========================================================

p = struct();

p.m__Q = 1.0;
p.m__L = 0.2;
p.L    = 0.5;
p.g    = 9.81;

% Tracking poles by output channel:
% channel 1: x_L1
% channel 2: x_L2
% channel 3: x_L3
poles_by_channel = {
    [-1.5 -1.5 -1.5 -1.5], ...
    [-1.5 -1.5 -1.5 -1.5], ...
    [-1.2 -1.2 -1.2 -1.2]
};

p = add_tracking_gains_from_poles(p, poles_by_channel);

% =========================================================
% INITIAL CONDITION
% State:
% [xP1 xP2 xP3 q1 q2 q3 vP1 vP2 vP3 omega1 omega2 omega3]'
% =========================================================

x0 = [
    0.20;    % xP1
   -0.15;    % xP2
   -0.65;    % xP3

    0.00;    % q1
    0.00;    % q2
    1.00;    % q3

    0.12;    % vP1
   -0.08;    % vP2
    0.05;    % vP3

    0.04;    % omega1
   -0.03;    % omega2
    0.02     % omega3
];

% =========================================================
% SIM OPTIONS
% =========================================================

opts = struct();

opts.solver  = @ode15s;
opts.RelTol  = 1e-8;
opts.AbsTol  = 1e-9;
opts.MaxStep = 1e-3;

opts.ref_output_rows = [1 2 3];
opts.output_indices  = [1 2 3];
opts.state_labels    = {'$x_{L1}$','$x_{L2}$','$x_{L3}$'};

opts.output_derivative_xdot_indices = {
    [1 2 3], ...   % ydot = xLdot = vL
    [7 8 9]        % yddot = vLdot from model substitution
};

opts.extra_state_groups = {
    struct('name','cable_direction_q', ...
    'indices',[4 5 6], ...
    'labels',{{'$q_1$','$q_2$','$q_3$'}}), ...

    struct('name','cable_angular_velocity_omega', ...
    'indices',[10 11 12], ...
    'labels',{{'$\omega_1$','$\omega_2$','$\omega_3$'}})
};

opts.save_dir = save_dir;
opts.title_prefix = '\textbf{Offset outer-loop QSFA effective-force $U$}';
opts.break_times = [];

% If using the step reference, use the switching times:
if strcmp(func2str(ref_fun), 'ref_regulation_steps')
    opts.break_times = [5, 20, 35, 50];
    tf = 65.0;
end

% =========================================================
% RUN SIMULATION
% =========================================================

fprintf('\n=========================================================\n');
fprintf('Running offset outer-loop effective-force simulation\n');
fprintf('Reference: %s\n', func2str(ref_fun));
fprintf('=========================================================\n');

out = simulate_closed_loop( ...
    model_fun, ...
    controller_fun, ...
    p, ...
    ref_fun, ...
    x0, ...
    tf, ...
    opts);

disp('RMSE:');
disp(out.rmse);

disp('Final abs err:');
disp(out.final_abs_err);

disp('Max abs input U = [U1 U2 U3]:');
disp(max(abs(out.U), [], 1));