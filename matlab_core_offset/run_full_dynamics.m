clear; close all; clc;

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

% Supported trajectory names:
%   'hover'
%   'fig8'
%   'fig8_zsin'
%   'helix'
%   'spiral'
%   'regulation_steps'
trajectory_name = 'fig8_zsin';

Tf = 40.0;
Ts = 0.01;

save_dir = fullfile(pwd, 'results_offset_fulldynamics', trajectory_name);
mat_name = fullfile(save_dir, 'sim_offset_QSFA_U.mat');

opts = struct();
opts.save_dir = save_dir;
opts.title_prefix = '\textbf{Offset full-dynamics QSFA effective-force $U$}';
opts.break_times = [];
opts.close_figures_after_save = false;
opts.save_figures = true;
opts.save_mat = true;
opts.save_summary = true;

% If using the step reference, use the switching times.
if strcmpi(trajectory_name, 'regulation_steps')
    opts.break_times = [5, 20, 35, 50];
    Tf = 65.0;
end

% Constant yaw command used by the mapper's flatness reference constructor.
psi_fun = @(t) 0;

% =========================================================
% MODEL, CONTROLLER, MAPPER, INNER LOOP
% =========================================================

model_fun = @model_offset;
outer_controller_fun = @controller_QSFA_U;
mapper_fun = @mapper_QSFA_U_to_inner;
inner_loop_fun = @inner_loop_offset;
ref_fun = make_reference_function(trajectory_name);

% =========================================================
% PARAMETERS
% =========================================================

p = struct();

p.m__Q = 1.0;
p.m__L = 0.2;
p.L    = 0.5;
p.g    = 9.81;

p.r = [0.25; 0; 0];
p.J = diag([0.02, 0.02, 0.04]);

p.r__1 = p.r(1);
p.r__2 = p.r(2);
p.r__3 = p.r(3);

p.J__1 = p.J(1,1);
p.J__2 = p.J(2,2);
p.J__3 = p.J(3,3);

% Tracking poles by output channel for controller_QSFA_U.
% The third channel uses the reduced controller's two required z gains.
poles_by_channel = {
    [-1.5 -1.5 -1.5 -1.5], ...
    [-1.5 -1.5 -1.5 -1.5], ...
    [-1.2 -1.2]
};

p = add_tracking_gains_from_poles(p, poles_by_channel);

% Aliases used by mapper_QSFA_U_to_inner and inner_loop_offset.
p.mQ = p.m__Q;
p.mL = p.m__L;
p.JS = p.J;

p.kR     = diag([4.0, 4.0, 4.0]);
p.kOmega = diag([1.2, 1.2, 1.2]);

% Keep these names too, in case older diagnostics use them.
p.k_R     = p.kR;
p.k_Omega = p.kOmega;

% =========================================================
% INITIAL CONDITION
% State:
% [x_L(3); q(3); R row-major(9); v_L(3); omega(3); Omega(3)]
% =========================================================

Rinit = ref_fun(0);

if size(Rinit,1) ~= 3 || size(Rinit,2) ~= 7
    error('Reference function must return a 3x7 matrix.');
end

% Same initial-load style as the outer-loop test, expressed as a perturbation
% around the initial reference position.
xL0 = Rinit(:,1) + [0.20; -0.15; -0.15];
q0 = [0; 0; 1];

R0 = eye(3);

vL0 = Rinit(:,2) + [0.12; -0.08; 0.05];
omega0 = [0.04; -0.03; 0.02];
Omega0 = [0; 0; 0];

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

x0 = project_state(x0);

% =========================================================
% SIMULATION STORAGE
% =========================================================

N = floor(Tf/Ts);
T = (0:N)'*Ts;

X = zeros(N+1, 24);
Xdot = zeros(N+1, 24);

Uplant = zeros(N+1, 4);

Ud_log = zeros(N+1, 3);
Fd_log = zeros(N+1, 3);
CU_log = zeros(N+1, 3);
Tscalar_log = zeros(N+1, 1);
eta_log = zeros(N+1, 3);

ut_log = zeros(N+1, 1);
tau_b_log = zeros(N+1, 3);
tau_cm_log = zeros(N+1, 3);

Rd_log = zeros(N+1, 9);
Rr_log = zeros(N+1, 9);
Rrdot_log = zeros(N+1, 9);
Rrddot_log = zeros(N+1, 9);

Omegad_log = zeros(N+1, 3);
dOmegad_log = zeros(N+1, 3);

ref_log = zeros(N+1, 3);
ref_vel_log = zeros(N+1, 3);
ref_acc_log = zeros(N+1, 3);
err_log = zeros(N+1, 3);
vel_err_log = zeros(N+1, 3);

xQ_log = zeros(N+1, 3);
xP_log = zeros(N+1, 3);

qnorm_log = zeros(N+1, 1);
omega_q_log = zeros(N+1, 1);
RU_det_log = zeros(N+1, 1);
RU_orth_log = zeros(N+1, 1);
Rr_det_log = zeros(N+1, 1);
Rr_orth_log = zeros(N+1, 1);
Omega_skew_log = zeros(N+1, 1);

Omega_log = zeros(N+1, 3);
q_log = zeros(N+1, 3);
omega_log = zeros(N+1, 3);

eR_log = zeros(N+1, 3);
eOmega_log = zeros(N+1, 3);

X(1,:) = x0.';

% =========================================================
% FIXED-STEP SAMPLED CLOSED LOOP
% Controller is evaluated once per sample.
% Plant is integrated with RK4 under held plant input.
% The state is projected after every accepted step.
% =========================================================

fprintf('\n=========================================================\n');
fprintf('Running offset full-dynamics QSFA-U simulation\n');
fprintf('Reference: %s\n', trajectory_name);
fprintf('Tf = %.3f s, Ts = %.5f s, N = %d\n', Tf, Ts, N);
fprintf('=========================================================\n\n');

for k = 1:N+1
    t = T(k);
    x = X(k,:).';

    Rt = ref_fun(t);

    [uplant, aux] = controller_step( ...
        t, x, p, ref_fun, psi_fun, ...
        outer_controller_fun, mapper_fun, inner_loop_fun);

    [fvec, G] = model_fun(x, p);
    xdot = fvec + G*uplant;

    Uplant(k,:) = uplant.';
    Xdot(k,:) = xdot.';

    Ud_log(k,:) = aux.Ud.';
    Fd_log(k,:) = aux.cmd.F_d.';
    CU_log(k,:) = aux.cmd.C_U_d.';
    Tscalar_log(k) = aux.cmd.Td_scalar;
    eta_log(k,:) = aux.cmd.eta_d.';

    ut_log(k) = aux.cmd.u_t_d;
    tau_b_log(k,:) = aux.inner_loop.tau_b.';
    tau_cm_log(k,:) = aux.inner_loop.tau_cm.';

    Rd_log(k,:) = reshape(aux.cmd.Rd.', 1, 9);
    Rr_log(k,:) = reshape(aux.map.Rr_d.', 1, 9);
    Rrdot_log(k,:) = reshape(aux.map.Rrdot_d.', 1, 9);
    Rrddot_log(k,:) = reshape(aux.map.Rrddot_d.', 1, 9);

    Omegad_log(k,:) = aux.cmd.Omega_d.';
    dOmegad_log(k,:) = aux.cmd.dOmega_d.';

    ref_log(k,:) = Rt(:,1).';
    ref_vel_log(k,:) = Rt(:,2).';
    ref_acc_log(k,:) = Rt(:,3).';

    err_log(k,:) = (x(1:3) - Rt(:,1)).';
    vel_err_log(k,:) = (x(16:18) - Rt(:,2)).';

    [xL_now, q_now, R_now, vL_now, omega_now, Omega_now] = unpack_state(x); %#ok<ASGLU>
    xP_now = xL_now - p.L*q_now;
    xQ_now = xP_now - R_now*p.r(:);

    xP_log(k,:) = xP_now.';
    xQ_log(k,:) = xQ_now.';

    q_log(k,:) = q_now.';
    omega_log(k,:) = omega_now.';
    Omega_log(k,:) = Omega_now.';

    qnorm_log(k) = norm(q_now);
    omega_q_log(k) = dot(omega_now, q_now);

    RU_det_log(k) = aux.map.RU_det;
    RU_orth_log(k) = aux.map.RU_orthogonality_error;
    Rr_det_log(k) = aux.map.Rr_det;
    Rr_orth_log(k) = aux.map.Rr_orthogonality_error;
    Omega_skew_log(k) = norm(aux.map.Omega_hat_d + aux.map.Omega_hat_d.', 'fro');

    eR_log(k,:) = aux.inner_loop.eR.';
    eOmega_log(k,:) = aux.inner_loop.eOmega.';

    if k <= N
        x_next = rk4_plant_step(x, uplant, p, Ts);
        x_next = project_state(x_next);
        X(k+1,:) = x_next.';
    end

    if mod(k, max(1, floor((N+1)/20))) == 0
        fprintf('  %.1f %%\n', 100*k/(N+1));
    end
end

% =========================================================
% PACK RESULTS
% =========================================================

sim = struct();

sim.T = T;
sim.X = X;
sim.Xdot = Xdot;

sim.Uplant = Uplant;
sim.Ud = Ud_log;

sim.Fd = Fd_log;
sim.C_U_d = CU_log;
sim.Td_scalar = Tscalar_log;
sim.eta_d = eta_log;

sim.u_t = ut_log;
sim.tau_b = tau_b_log;
sim.tau_cm = tau_cm_log;

sim.Rd = Rd_log;
sim.Rr = Rr_log;
sim.Rrdot = Rrdot_log;
sim.Rrddot = Rrddot_log;

sim.Omegad = Omegad_log;
sim.dOmegad = dOmegad_log;

sim.ref = ref_log;
sim.ref_vel = ref_vel_log;
sim.ref_acc = ref_acc_log;
sim.err = err_log;
sim.vel_err = vel_err_log;

sim.xQ = xQ_log;
sim.xP = xP_log;
sim.q = q_log;
sim.omega = omega_log;
sim.Omega = Omega_log;

sim.qnorm = qnorm_log;
sim.omega_dot_q = omega_q_log;
sim.RU_det = RU_det_log;
sim.RU_orthogonality_error = RU_orth_log;
sim.Rr_det = Rr_det_log;
sim.Rr_orthogonality_error = Rr_orth_log;
sim.Omega_hat_skew_error = Omega_skew_log;

sim.eR = eR_log;
sim.eOmega = eOmega_log;

sim.p = p;
sim.Tf = Tf;
sim.Ts = Ts;
sim.trajectory_name = trajectory_name;
sim.opts = opts;
sim.mat_name = mat_name;
sim.save_dir = save_dir;

sim.rmse = sqrt(mean(sim.err.^2, 1));
sim.max_abs_err = max(abs(sim.err), [], 1);
sim.final_abs_err = abs(sim.err(end,:));
sim.max_pos_err_norm = max(vecnorm(sim.err,2,2));
sim.final_pos_err_norm = norm(sim.err(end,:));
sim.max_eR_norm = max(vecnorm(sim.eR,2,2));
sim.final_eR_norm = norm(sim.eR(end,:));
sim.max_eOmega_norm = max(vecnorm(sim.eOmega,2,2));
sim.final_eOmega_norm = norm(sim.eOmega(end,:));

% =========================================================
% SAVE DATA, SUMMARY, AND FIGURES
% =========================================================

if ~isempty(save_dir) && ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

if opts.save_mat
    save(mat_name, 'sim');
end

fprintf('\nSimulation summary:\n');
fprintf('Final position error norm: %.6e\n', sim.final_pos_err_norm);
fprintf('Max position error norm:   %.6e\n', sim.max_pos_err_norm);
fprintf('Final attitude error norm: %.6e\n', sim.final_eR_norm);
fprintf('Max attitude error norm:   %.6e\n', sim.max_eR_norm);
fprintf('Max |u_t|:                 %.6e\n', max(abs(sim.u_t)));
fprintf('Max ||tau_cm||:            %.6e\n', max(vecnorm(sim.tau_cm,2,2)));

if opts.save_summary
    write_summary_file(sim, fullfile(save_dir, 'summary.txt'));
end

figs = plot_full_dynamics_results(sim, opts);
% ==========================================================
% Axis tick number formatting
% ==========================================================

% x_tick_format = '%.2f';
y_tick_format = '%.2f';

axs = findall(gcf, 'Type', 'axes');

for i = 1:numel(axs)
    ax = axs(i);
    % xtickformat(ax, x_tick_format);
    ytickformat(ax, y_tick_format);

end

if opts.save_figures
    save_all_figures(figs, save_dir, opts.close_figures_after_save);
end

if opts.save_mat
    fprintf('\nSaved simulation to: %s\n', mat_name);
end
fprintf('Saved results directory: %s\n\n', save_dir);

% =========================================================
% LOCAL FUNCTIONS
% =========================================================

function [uplant, aux] = controller_step( ...
    t, x_full, p, ref_fun, psi_fun, outer_controller_fun, mapper_fun, inner_loop_fun)

% Reduced state:
% xred = [xL; q; vL; omega]
xred = [
    x_full(1:6);
    x_full(16:21)
];

Ud = outer_controller_fun(t, xred, p, ref_fun);

Rt = ref_fun(t);
traj = make_traj_struct(Rt);
traj.psi = psi_fun(t);

[cmd, aux_map] = mapper_fun(t, xred, Ud, traj, p);

R_actual = R_from_full_state(x_full);
Omega_actual = x_full(22:24);

[uplant, aux_inner] = inner_loop_fun(R_actual, Omega_actual, cmd, p);

aux = struct();
aux.Ud = Ud;
aux.cmd = cmd;
aux.map = aux_map;
aux.inner_loop = aux_inner;
aux.traj = traj;

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

function x_next = rk4_plant_step(x, u, p, h)

k1 = plant_rhs(x, u, p);
k2 = plant_rhs(x + 0.5*h*k1, u, p);
k3 = plant_rhs(x + 0.5*h*k2, u, p);
k4 = plant_rhs(x + h*k3, u, p);

x_next = x + (h/6)*(k1 + 2*k2 + 2*k3 + k4);

end

function dx = plant_rhs(x, u, p)

[fvec, G] = model_offset(x, p);
dx = fvec + G*u;

end

function x = project_state(x)

q = x(4:6);
nq = norm(q);

if nq < 1e-12
    q = [0; 0; 1];
else
    q = q/nq;
end

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
omega = omega - q*dot(q, omega);
x(19:21) = omega;

end

function [xL, q, R, vL, omega, Omega] = unpack_state(x)

xL = x(1:3);
q = x(4:6);
R = R_from_full_state(x);
vL = x(16:18);
omega = x(19:21);
Omega = x(22:24);

end

function R = R_from_full_state(x)

R = [
    x(7),  x(8),  x(9);
    x(10), x(11), x(12);
    x(13), x(14), x(15)
];

end

function ref_fun = make_reference_function(name)

switch lower(name)
    case 'hover'
        ref_fun = @(t) ref_hover7(t, [0; 0; -0.5]);

    case 'fig8'
        ref_fun = @ref_fig8;

    case 'fig8_zsin'
        ref_fun = @ref_fig8_zsin;

    case 'helix'
        ref_fun = @ref_helix;

    case 'spiral'
        ref_fun = @ref_spiral;

    case 'regulation_steps'
        ref_fun = @ref_regulation_steps;

    otherwise
        error('Unknown trajectory_name: %s', name);
end

end

function R = ref_hover7(~, pos)

R = zeros(3,7);
R(:,1) = pos(:);

end

function figs = plot_full_dynamics_results(sim, opts)

apply_latex_defaults();

T = sim.T;
X = sim.X;
figs = gobjects(0);

labels = {'$x_{L1}$','$x_{L2}$','$x_{L3}$'};
use_stairs = ~isempty(opts.break_times);

% 1) 3D load trajectory
f1 = figure('Color','w','Name','traj3d');
plot3(sim.ref(:,1), sim.ref(:,2), sim.ref(:,3), '--', 'LineWidth', 1.5, ...
    'DisplayName', 'Reference load'); hold on;
plot3(X(:,1), X(:,2), X(:,3), '-', 'LineWidth', 1.5, ...
    'DisplayName', 'Actual load');
% plot3(sim.xQ(:,1), sim.xQ(:,2), sim.xQ(:,3), '-', 'LineWidth', 1.0, ...
%     'DisplayName', 'Quadrotor CoM');
grid on; axis equal;
xlabel('$x~[\mathrm{m}]$', 'Interpreter','latex');
ylabel('$y~[\mathrm{m}]$', 'Interpreter','latex');
zlabel('$z~[\mathrm{m}]$', 'Interpreter','latex');
legend('Interpreter','latex','Location','best');
view(35,25);
figs(end+1) = f1;

% 2) Position tracking
f2 = figure('Color','w','Name','tracking_position');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
for i = 1:3
    ax = nexttile;
    if use_stairs
        stairs(T, sim.ref(:,i), '--', 'LineWidth', 1.3, 'DisplayName','Reference'); hold on;
    else
        plot(T, sim.ref(:,i), '--', 'LineWidth', 1.3, 'DisplayName','Reference'); hold on;
    end
    plot(T, X(:,i), '-', 'LineWidth', 1.5, 'DisplayName','Actual');
    grid on;
    ylabel(labels{i}, 'Interpreter','latex');
    if i == 1
        legend(ax, 'Interpreter','latex','Location','best');
    end
    if i == 3
        xlabel('$t~[\mathrm{s}]$', 'Interpreter','latex');
    end
end
figs(end+1) = f2;

% 3) Position errors and error norm
f3 = figure('Color','w','Name','errors_position');
tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
for i = 1:3
    nexttile;
    plot(T, sim.err(:,i), 'LineWidth', 1.4);
    grid on;
    ylabel(sprintf('$e_{x,%d}$', i), 'Interpreter','latex');
end
nexttile;
plot(T, vecnorm(sim.err,2,2), 'LineWidth', 1.5);
grid on;
ylabel('$\|e_x\|_2$', 'Interpreter','latex');
xlabel('$t~[\mathrm{s}]$', 'Interpreter','latex');
figs(end+1) = f3;

% 4) Velocity tracking
f4 = figure('Color','w','Name','tracking_velocity');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
for i = 1:3
    ax = nexttile;
    plot(T, sim.ref_vel(:,i), '--', 'LineWidth', 1.3, 'DisplayName','Reference'); hold on;
    plot(T, X(:,15+i), '-', 'LineWidth', 1.5, 'DisplayName','Actual');
    grid on;
    ylabel(sprintf('$\\dot{x}_{L%d}$', i), 'Interpreter','latex');
    if i == 1
        legend(ax, 'Interpreter','latex','Location','best');
    end
    if i == 3
        xlabel('$t~[\mathrm{s}]$', 'Interpreter','latex');
    end
end
figs(end+1) = f4;

% 5) Inputs
f5 = figure('Color','w','Name','inputs');
tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
nexttile;
plot(T, sim.u_t, 'LineWidth', 1.4);
grid on;
ylabel('$u_t$', 'Interpreter','latex');
for i = 1:3
    nexttile;
    plot(T, sim.tau_cm(:,i), 'LineWidth', 1.3, 'DisplayName','$\\tau_{\mathrm{cm}}$'); hold on;
    plot(T, sim.tau_b(:,i), '--', 'LineWidth', 1.1, 'DisplayName','$\\tau_b$');
    grid on;
    ylabel(sprintf('$\\tau_%d$', i), 'Interpreter','latex');
    if i == 1
        legend('Interpreter','latex','Location','best');
    end
    if i == 3
        xlabel('$t~[\mathrm{s}]$', 'Interpreter','latex');
    end
end
figs(end+1) = f5;

% 6) Outer input and mapper force
f6 = figure('Color','w','Name','outer_mapper_commands');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
nexttile;
plot(T, sim.Ud, 'LineWidth', 1.2);
grid on;
ylabel('$U_d$', 'Interpreter','latex');
legend({'$U_1$','$U_2$','$U_3$'}, 'Interpreter','latex','Location','best');
nexttile;
plot(T, sim.Fd, 'LineWidth', 1.2);
grid on;
ylabel('$F_{U,d}$', 'Interpreter','latex');
legend({'$F_1$','$F_2$','$F_3$'}, 'Interpreter','latex','Location','best');
nexttile;
plot(T, sim.Td_scalar, 'LineWidth', 1.3);
grid on;
ylabel('$T_{d,\mathrm{scalar}}$', 'Interpreter','latex');
xlabel('$t~[\mathrm{s}]$', 'Interpreter','latex');
figs(end+1) = f6;

% 7) Attitude errors
f7 = figure('Color','w','Name','attitude_errors');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
nexttile;
plot(T, sim.eR, 'LineWidth', 1.2);
grid on;
ylabel('$e_R$', 'Interpreter','latex');
legend({'$e_{R1}$','$e_{R2}$','$e_{R3}$'}, 'Interpreter','latex','Location','best');
nexttile;
plot(T, vecnorm(sim.eR,2,2), 'LineWidth', 1.4);
grid on;
ylabel('$\|e_R\|_2$', 'Interpreter','latex');
nexttile;
plot(T, vecnorm(sim.eOmega,2,2), 'LineWidth', 1.4);
grid on;
ylabel('$\|e_\Omega\|_2$', 'Interpreter','latex');
xlabel('$t~[\mathrm{s}]$', 'Interpreter','latex');
figs(end+1) = f7;

% 8) Angular rates and desired angular acceleration
f8 = figure('Color','w','Name','angular_rates');
tiledlayout(4,1,'TileSpacing','compact','Padding','compact');

for i = 1:3
    nexttile;
    plot(T, sim.Omega(:,i), 'LineWidth', 1.25);
    hold on;
    plot(T, sim.Omegad(:,i), '--', 'LineWidth', 1.25);
    grid on;

    ylabel(sprintf('$\\Omega_%d$', i), 'Interpreter','latex');

    legend({sprintf('$\\Omega_%d$', i), sprintf('$\\Omega_{d%d}$', i)}, ...
        'Interpreter','latex', ...
        'Location','best');
end

nexttile;
plot(T, sim.dOmegad(:,1), 'LineWidth', 1.2);
hold on;
plot(T, sim.dOmegad(:,2), 'LineWidth', 1.2);
plot(T, sim.dOmegad(:,3), 'LineWidth', 1.2);
grid on;

ylabel('$\dot{\Omega}_d$', 'Interpreter','latex');
xlabel('$t~[\mathrm{s}]$', 'Interpreter','latex');

legend({'$\dot{\Omega}_{d1}$', '$\dot{\Omega}_{d2}$', '$\dot{\Omega}_{d3}$'}, ...
    'Interpreter','latex', ...
    'Location','best');

figs(end+1) = f8;


% 9) Cable states
f9 = figure('Color','w','Name','cable_states');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
nexttile;
plot(T, sim.q, 'LineWidth', 1.2);
grid on;
ylabel('$q$', 'Interpreter','latex');
legend({'$q_1$','$q_2$','$q_3$'}, 'Interpreter','latex','Location','best');
nexttile;
plot(T, sim.omega, 'LineWidth', 1.2);
grid on;
ylabel('$\omega$', 'Interpreter','latex');
legend({'$\omega_1$','$\omega_2$','$\omega_3$'}, 'Interpreter','latex','Location','best');
nexttile;
plot(T, sim.qnorm-1, 'LineWidth', 1.3); hold on;
plot(T, sim.omega_dot_q, '--', 'LineWidth', 1.3);
grid on;
ylabel('constraints', 'Interpreter','latex');
xlabel('$t~[\mathrm{s}]$', 'Interpreter','latex');
legend({'$\|q\|-1$','$\omega^Tq$'}, 'Interpreter','latex','Location','best');
figs(end+1) = f9;

% 10) Mapper/reference consistency
f10 = figure('Color','w','Name','mapper_consistency');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
nexttile;
plot(T, sim.RU_det, 'LineWidth', 1.3); hold on;
plot(T, sim.Rr_det, '--', 'LineWidth', 1.3);
grid on;
ylabel('$\\det(R)$', 'Interpreter','latex');
legend({'$R_{U,d}$','$R_{r,d}$'}, 'Interpreter','latex','Location','best');
nexttile;
semilogy(T, max(sim.RU_orthogonality_error, eps), 'LineWidth', 1.3); hold on;
semilogy(T, max(sim.Rr_orthogonality_error, eps), '--', 'LineWidth', 1.3);
grid on;
ylabel('$\|R^TR-I\|_F$', 'Interpreter','latex');
legend({'$R_{U,d}$','$R_{r,d}$'}, 'Interpreter','latex','Location','best');
nexttile;
semilogy(T, max(sim.Omega_hat_skew_error, eps), 'LineWidth', 1.3);
grid on;
ylabel('skew error', 'Interpreter','latex');
xlabel('$t~[\mathrm{s}]$', 'Interpreter','latex');
figs(end+1) = f10;

end

function write_summary_file(sim, file_name)

fid = fopen(file_name, 'w');
if fid < 0
    warning('Could not open summary file: %s', file_name);
    return;
end

fprintf(fid, 'Offset full-dynamics QSFA-U simulation\n');
fprintf(fid, 'Trajectory: %s\n', sim.trajectory_name);
fprintf(fid, 'Tf = %.6f s\n', sim.Tf);
fprintf(fid, 'Ts = %.6f s\n', sim.Ts);
fprintf(fid, '\n');
fprintf(fid, 'RMSE position             = [%g %g %g]\n', sim.rmse(1), sim.rmse(2), sim.rmse(3));
fprintf(fid, 'Max abs position error    = [%g %g %g]\n', sim.max_abs_err(1), sim.max_abs_err(2), sim.max_abs_err(3));
fprintf(fid, 'Final abs position error  = [%g %g %g]\n', sim.final_abs_err(1), sim.final_abs_err(2), sim.final_abs_err(3));
fprintf(fid, 'Max position error norm   = %g\n', sim.max_pos_err_norm);
fprintf(fid, 'Final position error norm = %g\n', sim.final_pos_err_norm);
fprintf(fid, 'Max attitude error norm   = %g\n', sim.max_eR_norm);
fprintf(fid, 'Final attitude error norm = %g\n', sim.final_eR_norm);
fprintf(fid, 'Max eOmega norm           = %g\n', sim.max_eOmega_norm);
fprintf(fid, 'Final eOmega norm         = %g\n', sim.final_eOmega_norm);
fprintf(fid, 'Max |u_t|                 = %g\n', max(abs(sim.u_t)));
fprintf(fid, 'Max ||tau_cm||            = %g\n', max(vecnorm(sim.tau_cm,2,2)));
fprintf(fid, 'Max ||tau_b||             = %g\n', max(vecnorm(sim.tau_b,2,2)));
fprintf(fid, 'Max |omega^T q|           = %g\n', max(abs(sim.omega_dot_q)));
fprintf(fid, 'Max |norm(q)-1|           = %g\n', max(abs(sim.qnorm-1)));
fprintf(fid, 'Max RU orth error         = %g\n', max(sim.RU_orthogonality_error));
fprintf(fid, 'Max Rr orth error         = %g\n', max(sim.Rr_orthogonality_error));
fprintf(fid, 'Max Omega skew error      = %g\n', max(sim.Omega_hat_skew_error));

fclose(fid);

end

function save_all_figures(figs, save_dir, close_after_save)

if isempty(save_dir)
    return;
end

if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

for k = 1:numel(figs)
    if ~ishandle(figs(k))
        continue;
    end

    nm = get(figs(k), 'Name');
    if isempty(nm)
        nm = sprintf('figure_%02d', k);
    end

    saveas(figs(k), fullfile(save_dir, [nm '.png']));
    savefig(figs(k), fullfile(save_dir, [nm '.fig']));

    if exist('saveFigureAsPDF', 'file') == 2
        try
            saveFigureAsPDF(figs(k), fullfile(save_dir, nm));
        catch ME
            warning('Could not save PDF for %s: %s', nm, ME.message);
        end
    end
end

if close_after_save
    close(figs(ishandle(figs)));
end

end

function apply_latex_defaults()

set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');

end
