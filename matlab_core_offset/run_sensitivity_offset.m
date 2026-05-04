clear; clc; close all;

addpath('simulationtools');

if exist('figureoptscall','file') == 2
    try
        figureoptscall;
    catch
    end
end

% =========================================================
% USER SETTINGS
% =========================================================

trajectory_name = 'fig8';

Tf = 30;
Ts = 0.01;

% Ten aggressive offset cases [rx ry rz] in body frame [m]
offset_cases = [
    0.00   0.00   0.00
    0.10   0.00   0.00
    0.20   0.00   0.00
    0.30   0.00   0.00
    0.40   0.00   0.00
    0.25   0.15   0.00
    0.25  -0.15   0.00
    0.25   0.00   0.15
    0.25   0.00  -0.15
    %0.35   0.20   0.15
];

results_root = fullfile(pwd, 'results_offset_sensitivity');
save_dir = fullfile(results_root, trajectory_name);

if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

save_results = true;
save_figures = true;

% =========================================================
% PARAMETERS
% =========================================================

p0 = make_params();

ref_fun = select_reference_function(trajectory_name);

% Initial reduced state
xL0 = ref_fun(0);
xL0 = xL0(:,1);

q0 = [0; 0; 1];
vL0 = [0; 0; 0];
omega0 = [0; 0; 0];

R0 = eye(3);
Omega0 = [0; 0; 0];

% Full state:
% x = [xL; q; R(: row-wise); vL; omega; Omega]
x0 = [
    xL0;
    q0;
    R0(1,1); R0(1,2); R0(1,3);
    R0(2,1); R0(2,2); R0(2,3);
    R0(3,1); R0(3,2); R0(3,3);
    vL0;
    omega0;
    Omega0
];

T = (0:Ts:Tf).';
N = numel(T);

% =========================================================
% STORAGE
% =========================================================

num_cases = size(offset_cases, 1);

case_data = cell(num_cases,1);

err_norm_all = zeros(N, num_cases);
err_x_all = zeros(N, num_cases);
err_y_all = zeros(N, num_cases);
err_z_all = zeros(N, num_cases);

final_err = zeros(num_cases,1);
rms_err = zeros(num_cases,1);
max_err = zeros(num_cases,1);

% =========================================================
% RUN CASES
% =========================================================

fprintf('\n==================================================\n');
fprintf('Offset sensitivity study\n');
fprintf('Trajectory: %s\n', trajectory_name);
fprintf('==================================================\n');

for icase = 1:num_cases

    r_vec = offset_cases(icase,:).';
    
    p = p0;
    p.r = r_vec;
    p.r__1 = p.r(1);
    p.r__2 = p.r(2);
    p.r__3 = p.r(3);

    fprintf('\nCase %d/%d: r = [%.3f, %.3f, %.3f]^T m\n', ...
        icase, num_cases, p.r(1), p.r(2), p.r(3));

    sim = run_one_full_case(T, Ts, x0, p, ref_fun, trajectory_name);

    case_data{icase} = sim;

    err = sim.err;

    err_x_all(:,icase) = err(:,1);
    err_y_all(:,icase) = err(:,2);
    err_z_all(:,icase) = err(:,3);
    err_norm_all(:,icase) = vecnorm(err, 2, 2);

    final_err(icase) = err_norm_all(end,icase);
    rms_err(icase)   = sqrt(mean(err_norm_all(:,icase).^2));
    max_err(icase)   = max(err_norm_all(:,icase));

    fprintf('  final error norm = %.6e m\n', final_err(icase));
    fprintf('  RMS error norm   = %.6e m\n', rms_err(icase));
    fprintf('  max error norm   = %.6e m\n', max_err(icase));
end

% =========================================================
% PACK RESULTS
% =========================================================

sensitivity = struct();

sensitivity.trajectory_name = trajectory_name;
sensitivity.T = T;
sensitivity.offset_cases = offset_cases;

sensitivity.err_x = err_x_all;
sensitivity.err_y = err_y_all;
sensitivity.err_z = err_z_all;
sensitivity.err_norm = err_norm_all;

sensitivity.final_err = final_err;
sensitivity.rms_err = rms_err;
sensitivity.max_err = max_err;

sensitivity.case_data = case_data;

if save_results
    mat_file = fullfile(save_dir, 'offset_sensitivity_results.mat');
    save(mat_file, 'sensitivity', '-v7.3');
    fprintf('\nSaved sensitivity MAT file:\n  %s\n', mat_file);
end

% =========================================================
% SUMMARY TABLE
% =========================================================

offset_norm = vecnorm(offset_cases, 2, 2);

summary_table = table( ...
    offset_cases(:,1), ...
    offset_cases(:,2), ...
    offset_cases(:,3), ...
    offset_norm, ...
    final_err, ...
    rms_err, ...
    max_err, ...
    'VariableNames', {'rx_m', 'ry_m', 'rz_m', 'r_norm_m', ...
                      'final_error_m', 'rms_error_m', 'max_error_m'} ...
);

disp(summary_table);
fprintf('\nCase map:\n');
for icase = 1:num_cases
    fprintf('  Case %d: r = [%.3f %.3f %.3f] m\n', ...
        icase, ...
        offset_cases(icase,1), ...
        offset_cases(icase,2), ...
        offset_cases(icase,3));
end

summary_file = fullfile(save_dir, 'offset_sensitivity_summary.txt');
fid = fopen(summary_file, 'w');

fprintf(fid, 'Offset sensitivity study\n');
fprintf(fid, 'Trajectory: %s\n\n', trajectory_name);
fprintf(fid, 'rx_m\try_m\trz_m\tr_norm_m\tfinal_error_m\trms_error_m\tmax_error_m\n');

for icase = 1:num_cases
    fprintf(fid, '%.6f\t%.6f\t%.6f\t%.6f\t%.12e\t%.12e\t%.12e\n', ...
        offset_cases(icase,1), ...
        offset_cases(icase,2), ...
        offset_cases(icase,3), ...
        offset_norm(icase), ...
        final_err(icase), ...
        rms_err(icase), ...
        max_err(icase));
end

fclose(fid);

fprintf('Saved summary file:\n  %s\n', summary_file);

% =========================================================
% PLOTS
% =========================================================

figs = plot_sensitivity_results(T, offset_cases, ...
    err_x_all, err_y_all, err_z_all, err_norm_all, ...
    final_err, rms_err, max_err);

if save_figures
    save_all_figures(figs, save_dir);
end

fprintf('\nSensitivity output directory:\n  %s\n\n', save_dir);

% =========================================================
% LOCAL FUNCTIONS
% =========================================================

function sim = run_one_full_case(T, Ts, x0, p, ref_fun, trajectory_name)

N = numel(T);
nx = numel(x0);

X = zeros(N,nx);
X(1,:) = x0.';

Uplant = zeros(N,4);
Ud_log = zeros(N,3);

ref_log = zeros(N,3);
err_log = zeros(N,3);

eR_log = zeros(N,3);
eOmega_log = zeros(N,3);

u_t_log = zeros(N,1);
tau_b_log = zeros(N,3);
tau_cm_log = zeros(N,3);

F_log = zeros(N,3);
Omega_d_log = zeros(N,3);
dOmega_d_log = zeros(N,3);

for k = 1:N

    t = T(k);
    x = X(k,:).';

    [uplant, aux] = controller_step(t, x, p, ref_fun);

    Uplant(k,:) = uplant.';

    Ud_log(k,:) = aux.Ud.';
    F_log(k,:) = aux.cmd.F_d.';
    u_t_log(k) = aux.cmd.u_t_d;

    tau_b_log(k,:) = aux.inner_loop.tau_b.';
    tau_cm_log(k,:) = aux.inner_loop.tau_cm.';

    eR_log(k,:) = aux.inner_loop.eR.';
    eOmega_log(k,:) = aux.inner_loop.eOmega.';

    Omega_d_log(k,:) = aux.cmd.Omega_d.';
    dOmega_d_log(k,:) = aux.cmd.dOmega_d.';

    Rt = ref_fun(t);
    ref_log(k,:) = Rt(:,1).';
    err_log(k,:) = (x(1:3) - Rt(:,1)).';

    if k < N
        x_next = rk4_plant_step(x, uplant, p, Ts);
        x_next = project_state(x_next);
        X(k+1,:) = x_next.';
    end
end

sim = struct();

sim.trajectory_name = trajectory_name;
sim.T = T;
sim.X = X;
sim.p = p;

sim.Uplant = Uplant;
sim.Ud = Ud_log;

sim.ref = ref_log;
sim.err = err_log;

sim.eR = eR_log;
sim.eOmega = eOmega_log;

sim.u_t = u_t_log;
sim.tau_b = tau_b_log;
sim.tau_cm = tau_cm_log;

sim.F_d = F_log;
sim.Omega_d = Omega_d_log;
sim.dOmega_d = dOmega_d_log;

sim.xL = X(:,1:3);

end

function [uplant, aux] = controller_step(t, x_full, p, ref_fun)

% Reduced state:
% xred = [xL; q; vL; omega]
xred = [
    x_full(1:6);
    x_full(16:21)
];

Ud = controller_QSFA_U(t, xred, p, ref_fun);

Rt = ref_fun(t);
traj = make_traj_struct(Rt);
traj.psi = 0;

[cmd, aux_map] = mapper_QSFA_U_to_inner(t, xred, Ud, traj, p);

R_actual = R_from_full_state(x_full);
Omega_actual = x_full(22:24);

[uplant, aux_inner] = inner_loop_offset(R_actual, Omega_actual, cmd, p);

aux = struct();
aux.Ud = Ud;
aux.cmd = cmd;
aux.map = aux_map;
aux.inner_loop = aux_inner;

end

function x_next = rk4_plant_step(x, u, p, h)

k1 = plant_rhs(x, u, p);
k2 = plant_rhs(x + 0.5*h*k1, u, p);
k3 = plant_rhs(x + 0.5*h*k2, u, p);
k4 = plant_rhs(x + h*k3, u, p);

x_next = x + (h/6)*(k1 + 2*k2 + 2*k3 + k4);

end

function xdot = plant_rhs(x, u, p)

[fvec, G] = model_offset(x, p);
xdot = fvec + G*u;

end

function x = project_state(x)

q = x(4:6);
q = q / norm(q);
x(4:6) = q;

R = R_from_full_state(x);

[U,~,V] = svd(R);
R = U*V.';

if det(R) < 0
    U(:,3) = -U(:,3);
    R = U*V.';
end

x(7:15) = [
    R(1,1); R(1,2); R(1,3);
    R(2,1); R(2,2); R(2,3);
    R(3,1); R(3,2); R(3,3)
];

omega = x(19:21);
omega = omega - q*dot(q,omega);
x(19:21) = omega;

end

function R = R_from_full_state(x)

R = [
    x(7),  x(8),  x(9);
    x(10), x(11), x(12);
    x(13), x(14), x(15)
];

end

function traj = make_traj_struct(Rt)

traj.xL0 = Rt(:,1);
traj.xL1 = Rt(:,2);
traj.xL2 = Rt(:,3);
traj.xL3 = Rt(:,4);
traj.xL4 = Rt(:,5);
traj.xL5 = Rt(:,6);
traj.xL6 = Rt(:,7);

end

function figs = plot_sensitivity_results(T, offset_cases, ...
    err_x_all, err_y_all, err_z_all, err_norm_all, ...
    final_err, rms_err, max_err)

num_cases = size(offset_cases, 1);

labels = cell(num_cases,1);

for i = 1:num_cases
    labels{i} = sprintf('Case %d', i);
end

figs = gobjects(0);

% ---------------------------------------------------------
% Error norm comparison
% ---------------------------------------------------------

f1 = figure('Color','w','Name','sensitivity_error_norm');
hold on; grid on;

for i = 1:num_cases
    plot(T, err_norm_all(:,i), 'LineWidth', 1.4);
end

xlabel('$t~[\mathrm{s}]$', 'Interpreter','latex');
ylabel('$\|e_x\|~[\mathrm{m}]$', 'Interpreter','latex');
legend(labels, 'Interpreter','latex', 'Location','best');
% title('Load Position Error Norm vs Offset', 'Interpreter','latex');

apply_axis_format(gcf);
figs(end+1) = f1;

% ---------------------------------------------------------
% Error components
% ---------------------------------------------------------

f2 = figure('Color','w','Name','sensitivity_error_components');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

component_data = {err_x_all, err_y_all, err_z_all};
component_labels = {'$e_x~[\mathrm{m}]$', '$e_y~[\mathrm{m}]$', '$e_z~[\mathrm{m}]$'};

 for j = 1:3
    nexttile;
    hold on; grid on;

    E = component_data{j};

    for i = 1:num_cases
        plot(T, E(:,i), 'LineWidth', 1.2);
    end

    ylabel(component_labels{j}, 'Interpreter','latex');

    if j == 1
        % legend(labels, 'Interpreter','latex', 'Location','best');
        % title('Load Position Error Components vs Offset', 'Interpreter','latex');
    end

    if j == 3
        xlabel('$t~[\mathrm{s}]$', 'Interpreter','latex');
    end
end

apply_axis_format(gcf);
figs(end+1) = f2;

% ---------------------------------------------------------
% Summary bar plot
% ---------------------------------------------------------

f3 = figure('Color','w','Name','sensitivity_summary_bars');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

case_names = strings(num_cases,1);

for i = 1:num_cases
    case_names(i) = sprintf('Case %d', i);
end

case_cat = categorical(case_names);
case_cat = reordercats(case_cat, cellstr(case_names));

nexttile;
bar(case_cat, rms_err);
grid on;
ylabel('RMS [m]');
% title('RMS Error vs Offset Case');
xtickangle(35);

nexttile;
bar(case_cat, max_err);
grid on;
ylabel('Max [m]');
xtickangle(35);

nexttile;
bar(case_cat, final_err);
grid on;
ylabel('Final [m]');
xlabel('Offset case');
xtickangle(35);

apply_axis_format(gcf);
figs(end+1) = f3;

end

function save_all_figures(figs, save_dir)

for i = 1:numel(figs)
    fig = figs(i);

    if ~isgraphics(fig)
        continue;
    end

    name = get(fig, 'Name');

    if isempty(name)
        name = sprintf('figure_%02d', i);
    end

    png_file = fullfile(save_dir, [name '.png']);
    fig_file = fullfile(save_dir, [name '.fig']);
    pdf_base = fullfile(save_dir, name);

    saveas(fig, png_file);
    savefig(fig, fig_file);

    if exist('saveFigureAsPDF', 'file') == 2
        try
            saveFigureAsPDF(fig, pdf_base);
        catch ME
            warning('Could not save PDF for %s: %s', name, ME.message);
        end
    end
end

end

function apply_axis_format(fig_handle)

if exist('figureoptscall','file') == 2
    try
        figure(fig_handle);
        figureoptscall;
    catch
    end
end

axs = findall(fig_handle, 'Type', 'axes');

for i = 1:numel(axs)
    ax = axs(i);

    try
        xtickformat(ax, '%.1f');
        ytickformat(ax, '%.3f');
    catch
    end
end

end

function ref_fun = select_reference_function(name)

switch lower(name)
    case {'hover'}
        ref_fun = @ref_hover;

    case {'regulation', 'regulation_steps', 'steps'}
        ref_fun = @ref_regulation_steps;

    case {'fig8', 'figure8', 'figure_eight'}
        ref_fun = @ref_fig8;

    case {'helix'}
        ref_fun = @ref_helix;

    otherwise
        error('Unknown trajectory_name: %s', name);
end

end

function p = make_params()

p = struct();

% Physical parameters
p.m__Q = 1.0;
p.m__L = 0.2;
p.L    = 0.5;
p.g    = 9.81;

p.r = [0.20; 0; 0];

p.J = diag([0.02, 0.02, 0.04]);

p.r__1 = p.r(1);
p.r__2 = p.r(2);
p.r__3 = p.r(3);

p.J__1 = p.J(1,1);
p.J__2 = p.J(2,2);
p.J__3 = p.J(3,3);

% Outer-loop gains
if exist('add_tracking_gains_from_poles','file') == 2
    p = add_tracking_gains_from_poles(p, {
        [-1.5 -1.5 -1.5 -1.5], ...
        [-1.5 -1.5 -1.5 -1.5], ...
        [-1.2 -1.2]
    });
end

% Aliases for mapper and inner loop
p.mQ = p.m__Q;
p.mL = p.m__L;
p.JS = p.J;

% Inner-loop gains
p.kR = diag([4.0, 4.0, 4.0]);
p.kOmega = diag([1.2, 1.2, 1.2]);

end