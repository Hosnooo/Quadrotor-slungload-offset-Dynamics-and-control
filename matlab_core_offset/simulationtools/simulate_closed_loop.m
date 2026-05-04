function out = simulate_closed_loop(model_fun, controller_fun, p, ref, x0, tf, opts)
% Generic closed-loop simulator for Maple-generated model/controller
%
% model_fun      : @(x,p) -> [fvec,G]
% controller_fun : @(t,x,p,ref) -> u
% ref            : @(t) -> ny x nrefcols matrix
%
% This version:
%   - no event logic
%   - no plotting flags
%   - always computes tracking levels
%   - always plots all standard figures
%   - plots extra state groups when provided

if nargin < 7
    opts = struct();
end

if ~isfield(opts,'solver'),             opts.solver = @ode15s; end
if ~isfield(opts,'RelTol'),             opts.RelTol = 1e-8; end
if ~isfield(opts,'AbsTol'),             opts.AbsTol = 1e-9; end
if ~isfield(opts,'MaxStep'),            opts.MaxStep = 1e-3; end
if ~isfield(opts,'output_indices'),     opts.output_indices = [1 2 3]; end
if ~isfield(opts,'state_labels'),       opts.state_labels = {'$x_{Q1}$','$x_{Q2}$','$x_{Q3}$'}; end
if ~isfield(opts,'save_dir'),           opts.save_dir = ''; end
if ~isfield(opts,'title_prefix'),       opts.title_prefix = ''; end
if ~isfield(opts,'break_times'),        opts.break_times = []; end
if ~isfield(opts,'extra_state_groups'), opts.extra_state_groups = {}; end
if ~isfield(opts,'ref_output_rows'),    opts.ref_output_rows = 1:numel(opts.output_indices); end
if ~isfield(opts,'output_derivative_xdot_indices'), opts.output_derivative_xdot_indices = {opts.output_indices}; end
if exist('figureoptscall','file') == 2
    try
        figureoptscall;
    catch
    end
end

apply_latex_defaults();

ode_opts = odeset( ...
    'RelTol', opts.RelTol, ...
    'AbsTol', opts.AbsTol, ...
    'MaxStep', opts.MaxStep);

rhs = @(t,x) closed_loop_rhs(t,x,model_fun,controller_fun,p,ref);

breaks = opts.break_times(:).';
breaks = breaks(breaks > 0 & breaks < tf);
breaks = unique(breaks);
edges = [0, breaks, tf];

t_all = [];
X_all = [];
x_init = x0(:);

for seg = 1:(numel(edges)-1)
    tspan = [edges(seg), edges(seg+1)];
    [t_seg, X_seg] = opts.solver(rhs, tspan, x_init, ode_opts);

    if seg > 1 && ~isempty(t_seg)
        t_seg = t_seg(2:end);
        X_seg = X_seg(2:end,:);
    end

    t_all = [t_all; t_seg];
    X_all = [X_all; X_seg];

    if ~isempty(X_seg)
        x_init = X_seg(end,:).';
    end
end

t = t_all;
X = X_all;

if isempty(t)
    error('Simulation returned empty time/state arrays.');
end

ny = numel(opts.output_indices);

Rt0_full = ref(t(1));
Rt0 = Rt0_full(opts.ref_output_rows,:);
nrefcols = size(Rt0, 2);

u0 = controller_fun(t(1), X(1,:).', p, ref);
nu = numel(u0);

Rcube = zeros(length(t), ny, nrefcols);
U = zeros(length(t), nu);
E = zeros(length(t), ny);
Xdot = zeros(length(t), size(X,2));

for k = 1:length(t)
    xk = X(k,:).';
    Rt_full = ref(t(k));
    Rt = Rt_full(opts.ref_output_rows,:);
    uk = controller_fun(t(k), xk, p, ref);
    [fvec, G] = model_fun(xk, p);
    xdotk = fvec + G*uk;

    Rcube(k,:,:) = Rt;
    U(k,:) = uk(:).';
    Xdot(k,:) = xdotk(:).';
    E(k,:) = X(k, opts.output_indices) - Rt(:,1).';
end

% ------------------------------------------------------------
% Tracking levels from model substitution only.
% No numerical gradients are used here.
%
% opts.output_derivative_xdot_indices{ell} gives the state-derivative
% indices in Xdot that correspond to y^(ell).
% ------------------------------------------------------------
max_level = min(numel(opts.output_derivative_xdot_indices), nrefcols - 1);

Ylevels = cell(max_level+1,1);
Rlevels = cell(max_level+1,1);
Elevels = cell(max_level+1,1);
level_rmse = cell(max_level+1,1);

% level 0: output itself
Ylevels{1} = X(:, opts.output_indices);
Rlevels{1} = Rcube(:,:,1);
Elevels{1} = Ylevels{1} - Rlevels{1};
level_rmse{1} = sqrt(mean(Elevels{1}.^2,1));

% higher levels: only from substituted dynamic model Xdot
for ell = 1:max_level
    idx = opts.output_derivative_xdot_indices{ell};

    if numel(idx) ~= ny
        error('Derivative index set for level %d has %d entries, but ny = %d.', ...
            ell, numel(idx), ny);
    end

    if max(idx) > size(Xdot,2)
        error('Derivative index set for level %d contains an index larger than the state dimension.', ell);
    end

    Ylevels{ell+1} = Xdot(:, idx);
    Rlevels{ell+1} = Rcube(:,:,ell+1);
    Elevels{ell+1} = Ylevels{ell+1} - Rlevels{ell+1};
    level_rmse{ell+1} = sqrt(mean(Elevels{ell+1}.^2,1));
end

out = struct();
out.t = t;
out.X = X;
out.Xdot = Xdot;
out.R = Rcube(:,:,1);
out.U = U;
out.E = E;
out.Enorm = vecnorm(E,2,2);

out.Ylevels = Ylevels;
out.Rlevels = Rlevels;
out.Elevels = Elevels;
out.level_rmse = level_rmse;
out.max_tracking_level = max_level;

out.rmse = sqrt(mean(E.^2,1));
out.max_abs_err = max(abs(E),[],1);
out.final_abs_err = abs(E(end,:));

tail_idx = max(1, round(0.9*length(t)));
tail_mean_abs = mean(abs(E(tail_idx:end,:)),1);
out.conv_axes = tail_mean_abs < 0.02;
out.conv_all = all(out.conv_axes);

figs = plot_closed_loop_results(out, opts);

if ~isempty(opts.save_dir)
    if ~exist(opts.save_dir,'dir')
        mkdir(opts.save_dir);
    end

    save(fullfile(opts.save_dir,'sim_data.mat'),'out');

    fid = fopen(fullfile(opts.save_dir,'summary.txt'),'w');
    fprintf(fid,'Simulation finished at t = %.6f s\n', t(end));
    fprintf(fid,'max(|u(1)|)      = %.6g\n', max(abs(U(:,1))));
    if size(U,2) >= 2
        fprintf(fid,'max(|u(2)|)      = %.6g\n', max(abs(U(:,2))));
    end
    if size(U,2) >= 3
        fprintf(fid,'max(|u(3)|)      = %.6g\n', max(abs(U(:,3))));
    end
    fprintf(fid,'RMSE             = [%g %g %g]\n', out.rmse(1), out.rmse(2), out.rmse(3));
    fprintf(fid,'Final abs err    = [%g %g %g]\n', out.final_abs_err(1), out.final_abs_err(2), out.final_abs_err(3));
    fprintf(fid,'Convergence      = [%d %d %d], overall=%d\n', ...
        out.conv_axes(1), out.conv_axes(2), out.conv_axes(3), out.conv_all);

    for ell = 0:max_level
        rmse_ell = out.level_rmse{ell+1};
        fprintf(fid,'Level %d RMSE     = [%g %g %g]\n', ell, rmse_ell(1), rmse_ell(2), rmse_ell(3));
    end
    fclose(fid);

    for k = 1:numel(figs)
        if ~ishandle(figs(k))
            continue;
        end
        nm = get(figs(k),'Name');
        if isempty(nm)
            nm = sprintf('figure_%02d',k);
        end
        saveas(figs(k), fullfile(opts.save_dir,[nm '.png']));
        savefig(figs(k), fullfile(opts.save_dir,[nm '.fig']));
        if exist('saveFigureAsPDF','file') == 2
            try
                saveFigureAsPDF(figs(k), fullfile(opts.save_dir, nm));
            catch
            end
        end
    end
    close(figs(ishandle(figs)));
end
end

function xdot = closed_loop_rhs(t,x,model_fun,controller_fun,p,ref)
u = controller_fun(t,x,p,ref);
[fvec,G] = model_fun(x,p);
xdot = fvec + G*u;
end

function figs = plot_closed_loop_results(out, opts)

labels = opts.state_labels;
tracking_labels = {
    {'$x_{Q1}$', '$x_{Q2}$', '$x_{Q3}$'}, ...
    {'$\dot{x}_{Q1}$', '$\dot{x}_{Q2}$', '$\dot{x}_{Q3}$'}, ...
    {'$\ddot{x}_{Q1}$', '$\ddot{x}_{Q2}$', '$\ddot{x}_{Q3}$'}, ...
    {'$x_{Q1}^{(3)}$', '$x_{Q2}^{(3)}$', '$x_{Q3}^{(3)}$'}
    };
figs = gobjects(0);

level_names = {'position','velocity','acceleration','jerk'};

use_stairs = ~isempty(opts.break_times);

% 1) 3D trajectory
f1 = figure('Color','w','Name','traj3d');
plot3(out.R(:,1), out.R(:,2), out.R(:,3), '--', 'LineWidth', 1.5, ...
    'DisplayName', 'Reference'); hold on;
plot3(out.X(:,opts.output_indices(1)), out.X(:,opts.output_indices(2)), out.X(:,opts.output_indices(3)), ...
    '-', 'LineWidth', 1.5, 'DisplayName', 'Actual');
grid on;
xlabel(labels{1}, 'Interpreter', 'latex');
ylabel(labels{2}, 'Interpreter', 'latex');
zlabel(labels{3}, 'Interpreter', 'latex');
legend('Interpreter', 'latex', 'Location', 'best');
view(35,25);
figs(end+1) = f1;

% 2) Position tracking
f2 = figure('Color','w','Name','tracking_position');
tl = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

for i = 1:3
    ax = nexttile;
    if use_stairs
        stairs(out.t, out.Rlevels{1}(:,i), '--', 'LineWidth', 1.3, 'DisplayName', 'Reference'); hold on;
    else
        plot(out.t, out.Rlevels{1}(:,i), '--', 'LineWidth', 1.3, 'DisplayName', 'Reference'); hold on;
    end
    plot(out.t, out.Ylevels{1}(:,i), '-', 'LineWidth', 1.5, 'DisplayName', 'Actual');
    grid on;
    ylabel(labels{i}, 'Interpreter', 'latex');
    if i == 1
        legend(ax, 'Interpreter', 'latex', 'Location', 'best');
    end
    if i == 3
        xlabel('$t~[\mathrm{s}]$', 'Interpreter', 'latex');
    end
end
%title(tl, [opts.title_prefix ' position tracking'], 'Interpreter', 'latex');
figs(end+1) = f2;

% 3) Position errors
f3 = figure('Color','w','Name','errors_position');
tl = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

for i = 1:3
    nexttile;
    plot(out.t, out.Elevels{1}(:,i), 'LineWidth', 1.4);
    grid on;
    ylabel(['$e_{', num2str(i), '}$'], 'Interpreter', 'latex');
    if i == 3
        xlabel('$t~[\mathrm{s}]$', 'Interpreter', 'latex');
    end
end
%title(tl, [opts.title_prefix ' position errors'], 'Interpreter', 'latex');
figs(end+1) = f3;

% 4) Higher tracking levels
for ell = 1:out.max_tracking_level
    nm = sprintf('tracking_%s', level_names{ell+1});
    fL = figure('Color','w','Name',nm);
    tl = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

    for i = 1:3
        ax = nexttile;
        if use_stairs
            stairs(out.t, out.Rlevels{ell+1}(:,i), '--', 'LineWidth', 1.3, 'DisplayName', 'Reference'); hold on;
        else
            plot(out.t, out.Rlevels{ell+1}(:,i), '--', 'LineWidth', 1.3, 'DisplayName', 'Reference'); hold on;
        end
        plot(out.t, out.Ylevels{ell+1}(:,i), '-', 'LineWidth', 1.5, 'DisplayName', 'Actual');
        grid on;
        ylabel(tracking_labels{ell+1}{i}, 'Interpreter', 'latex');
        if i == 1
            legend(ax, 'Interpreter', 'latex', 'Location', 'best');
        end
        if i == 3
            xlabel('$t~[\mathrm{s}]$', 'Interpreter', 'latex');
        end
    end

    %title(tl, [opts.title_prefix ' ' level_names{ell+1} ' tracking'], 'Interpreter', 'latex');
    figs(end+1) = fL;
end

% 5) Tracking-level error norms
fN = figure('Color','w','Name','tracking_level_error_norms');
tl = tiledlayout(out.max_tracking_level+1,1,'TileSpacing','compact','Padding','compact');

for ell = 0:out.max_tracking_level
    nexttile;
    plot(out.t, vecnorm(out.Elevels{ell+1},2,2), 'LineWidth', 1.4);
    grid on;
    ylabel(sprintf('$\\|e^{(%d)}\\|_2$', ell), 'Interpreter', 'latex');
    if ell == out.max_tracking_level
        xlabel('$t~[\mathrm{s}]$', 'Interpreter', 'latex');
    end
end
%title(tl, [opts.title_prefix ' tracking-level error norms'], 'Interpreter', 'latex');
figs(end+1) = fN;

% 6) Performance / control
f4 = figure('Color','w','Name','performance');
tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(out.t, out.Enorm, 'LineWidth', 1.5);
grid on;
ylabel('$\|e\|_2$', 'Interpreter', 'latex');

ax = nexttile;
for i = 1:size(out.U,2)
    plot(out.t, out.U(:,i), 'LineWidth', 1.2, ...
        'DisplayName', ['$u_', num2str(i), '$']); hold on;
end
grid on;
ylabel('$u$', 'Interpreter', 'latex');
xlabel('$t~[\mathrm{s}]$', 'Interpreter', 'latex');
legend(ax, 'Interpreter', 'latex', 'Location', 'best');

%title(tl, [opts.title_prefix ' performance'], 'Interpreter', 'latex');
figs(end+1) = f4;

% 7) Extra grouped state plots
for g = 1:numel(opts.extra_state_groups)
    grp = opts.extra_state_groups{g};
    figs(end+1) = plot_state_group_figure(out, grp, opts); %#ok<AGROW>
end
end

function fig = plot_state_group_figure(out, grp, opts)

fig = figure('Color','w','Name',grp.name);
nplot = numel(grp.indices);

if isfield(grp,'layout')
    nrow = grp.layout(1);
    ncol = grp.layout(2);
else
    nrow = nplot;
    ncol = 1;
end

tl = tiledlayout(nrow,ncol,'TileSpacing','compact','Padding','compact');

for i = 1:nplot
    nexttile;
    plot(out.t, out.X(:, grp.indices(i)), 'LineWidth', 1.4);
    grid on;
    ylabel(grp.labels{i}, 'Interpreter', 'latex');

    if i > (nrow-1)*ncol
        xlabel('$t~[\mathrm{s}]$', 'Interpreter', 'latex');
    end
end

%title(tl, [opts.title_prefix ' ' strrep(grp.name,'_',' ')], 'Interpreter', 'latex');
end

function apply_latex_defaults()
set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
end
