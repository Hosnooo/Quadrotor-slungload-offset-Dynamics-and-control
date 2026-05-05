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

trajectory_name = 'fig8_rough';

results_root = fullfile(pwd, 'results_offset_fulldynamics');
mat_name = fullfile(results_root, trajectory_name, 'sim_offset_QSFA_U.mat');

video_basename = 'offset_slung_load_animation';

% Video settings
T_start = 0;
T_end   = 40;

frame_rate = 30;          % smooth video
playback_speed = 1;       % simulation seconds per video second
video_quality = 95;       % MP4 quality

trail_seconds = inf;      % inf = full trail
z_invert_for_display = true;

save_video = true;
save_final_frame = true;
close_after_save = true;

% =========================================================
% SYNCHRONIZED PLOT VIDEOS
% =========================================================

save_plot_videos = true;

% This script creates these as separate synced videos:
%   offset_slung_load_animation_errors_position.mp4
%   offset_slung_load_animation_inputs.mp4
%   offset_slung_load_animation_attitude_errors.mp4
plot_video_names = {
    'errors_position'
    'inputs'
    'attitude_errors'
};

plot_cursor_color = [1 0 0];   % red vertical line
plot_cursor_width = 1.8;
plot_cursor_style = '--';

plot_video_position = [120 120 900 650];

% Visual scale settings
arm_len = 0.18;
prop_radius = 0.045;
load_radius = 0.03;
com_radius = 0.035;
attach_radius = 0.027;

% Visual-only rotation of quad arms about body z-axis b3.
% This does not change the true attitude, CoM, suspension point, or offset arm.
arm_yaw_visual = deg2rad(35);

% Use the current state variables q and R to reconstruct the pendulum
% suspension point and drone CoM in every frame.
use_state_derived_geometry = true;

% Extra attitude cues make drone rotation visible even when the quad cross is
% nearly symmetric.
show_body_axes = true;
body_axis_len = 0.14;

% =========================================================
% LOAD SAVED SIMULATION
% =========================================================

if ~exist(mat_name, 'file')
    fallback = fullfile(pwd, 'sim_offset_QSFA_U.mat');
    if exist(fallback, 'file')
        mat_name = fallback;
    else
        error('Animation:missingMatFile', ...
            'Could not find simulation MAT file: %s', mat_name);
    end
end

S = load(mat_name, 'sim');
sim = S.sim;

T = sim.T(:);
X = sim.X;
p = sim.p;

if isfield(sim, 'trajectory_name')
    trajectory_name = sim.trajectory_name;
end

if isfield(sim, 'save_dir') && ~isempty(sim.save_dir)
    save_dir = fullfile(sim.save_dir, 'animation');
else
    save_dir = fullfile(fileparts(mat_name), 'animation');
end

if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

video_mp4 = fullfile(save_dir, [video_basename '.mp4']);
video_avi = fullfile(save_dir, [video_basename '.avi']);

L = p.L;
r = p.r(:);

% =========================================================
% PRECOMPUTE GEOMETRY
% =========================================================

n = numel(T);

xL_all = X(:,1:3);

R_all = zeros(3,3,n);
q_all = zeros(n,3);
xP_from_state = zeros(n,3);
xQ_from_state = zeros(n,3);

for k = 1:n
    [xL, q, R] = unpack_full_state(X(k,:).');

    q_all(k,:) = q.';
    R_all(:,:,k) = R;

    % Reconstruct geometry from the current simulated states.
    % The pendulum link is xP -> xL = L*q.
    % The offset arm is xQ -> xP = R*r.
    xP = xL - L*q;
    xQ = xP - R*r;

    xP_from_state(k,:) = xP.';
    xQ_from_state(k,:) = xQ.';
end

if use_state_derived_geometry
    xP_all = xP_from_state;
    xQ_all = xQ_from_state;
else
    if isfield(sim, 'xP')
        temp_xP = normalize_signal_matrix(sim.xP, n, 3);
        if all(isfinite(temp_xP(:)))
            xP_all = temp_xP;
        else
            xP_all = xP_from_state;
        end
    else
        xP_all = xP_from_state;
    end

    if isfield(sim, 'xQ')
        temp_xQ = normalize_signal_matrix(sim.xQ, n, 3);
        if all(isfinite(temp_xQ(:)))
            xQ_all = temp_xQ;
        else
            xQ_all = xQ_from_state;
        end
    else
        xQ_all = xQ_from_state;
    end
end

if isfield(sim, 'ref')
    ref_all = normalize_signal_matrix(sim.ref, n, 3);
    if ~all(isfinite(ref_all(:)))
        ref_all = xL_all;
    end
else
    ref_all = xL_all;
end

% Build plotting data directly from the .mat simulation result.
plot_data = build_plot_data(sim, T, xL_all, ref_all);

xL_v = viz_points(xL_all, z_invert_for_display);
xP_v = viz_points(xP_all, z_invert_for_display);
xQ_v = viz_points(xQ_all, z_invert_for_display);
ref_v = viz_points(ref_all, z_invert_for_display);

all_pts = [xL_v; xP_v; xQ_v; ref_v];
lims = compute_equal_lims(all_pts, 0.09);

% =========================================================
% FRAME SELECTION
% =========================================================

idx = find(T >= T_start & T <= T_end);

if isempty(idx)
    error('Animation:emptyTimeWindow', ...
        'No simulation samples found between T_start = %.3f and T_end = %.3f.', ...
        T_start, T_end);
end

Ts_sim = mean(diff(T));
sim_dt_per_video_frame = playback_speed / frame_rate;
frame_step = max(1, round(sim_dt_per_video_frame / Ts_sim));

frame_ids = idx(1:frame_step:end);

if frame_ids(end) ~= idx(end)
    frame_ids(end+1) = idx(end);
end

if isfinite(trail_seconds)
    trail_samples = max(2, round(trail_seconds/Ts_sim));
else
    trail_samples = inf;
end

fprintf('\nAnimation time window: %.2f s to %.2f s\n', T(frame_ids(1)), T(frame_ids(end)));
fprintf('Number of synchronized frames: %d\n', numel(frame_ids));
fprintf('Approximate output video duration: %.2f s\n', numel(frame_ids)/frame_rate);

% =========================================================
% MAIN 3D FIGURE SETUP
% =========================================================

fig = figure('Color','w', 'Position',[80 80 1280 720], ...
    'Name','offset_slung_load_animation');

ax = axes('Parent',fig);
set(ax, 'Position', [0.07 0.08 0.82 0.86]);

hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');
box(ax,'on');
view(ax, 38, 24);

xlabel(ax, '$x~[\mathrm{m}]$', 'Interpreter','latex');
ylabel(ax, '$y~[\mathrm{m}]$', 'Interpreter','latex');

if z_invert_for_display
    zlabel(ax, '$-z~[\mathrm{m}]$', 'Interpreter','latex');
else
    zlabel(ax, '$z~[\mathrm{m}]$', 'Interpreter','latex');
end

xlim(ax, lims(1,:));
ylim(ax, lims(2,:));
zlim(ax, lims(3,:));

plot3(ax, ref_v(:,1), ref_v(:,2), ref_v(:,3), '--', ...
    'LineWidth', 1.25, 'Color', [0.35 0.35 0.35], ...
    'DisplayName','reference load path');

load_trail = plot3(ax, nan, nan, nan, '-', ...
    'LineWidth', 2.0, 'Color', [0.0 0.25 0.85], ...
    'DisplayName','actual load path');

com_trail = plot3(ax, nan, nan, nan, '-', ...
    'LineWidth', 1.35, 'Color', [0.85 0.10 0.10], ...
    'DisplayName','quad CoM path');

cable_line = plot3(ax, nan, nan, nan, '-', ...
    'LineWidth', 2.4, 'Color', [0.02 0.02 0.02], ...
    'DisplayName','cable');

offset_line = plot3(ax, nan, nan, nan, '-', ...
    'LineWidth', 3.0, 'Color', [0.85 0.1 0.1], ...
    'DisplayName','offset arm');

arm1_line = plot3(ax, nan, nan, nan, '-', ...
    'LineWidth', 4.2, 'Color', [0.05 0.05 0.05], ...
    'HandleVisibility','off');

arm2_line = plot3(ax, nan, nan, nan, '-', ...
    'LineWidth', 4.2, 'Color', [0.05 0.05 0.05], ...
    'HandleVisibility','off');

prop_lines = gobjects(4,1);
for i = 1:4
    prop_lines(i) = plot3(ax, nan, nan, nan, '-', ...
        'LineWidth', 1.5, 'Color', [0.05 0.05 0.05], ...
        'HandleVisibility','off');
end

body_x_line = plot3(ax, nan, nan, nan, '-', ...
    'LineWidth', 3.0, 'Color', [0.85 0.10 0.10], ...
    'HandleVisibility','off');

body_y_line = plot3(ax, nan, nan, nan, '-', ...
    'LineWidth', 3.0, 'Color', [0.10 0.55 0.10], ...
    'HandleVisibility','off');

body_z_line = plot3(ax, nan, nan, nan, '-', ...
    'LineWidth', 3.0, 'Color', [0.10 0.20 0.90], ...
    'HandleVisibility','off');

[sx, sy, sz] = sphere(28);

load_sphere = surf(ax, nan(size(sx)), nan(size(sy)), nan(size(sz)), ...
    'FaceColor',[0.10 0.10 0.10], 'EdgeColor','none', ...
    'FaceAlpha',0.96, 'HandleVisibility','off');

com_sphere = surf(ax, nan(size(sx)), nan(size(sy)), nan(size(sz)), ...
    'FaceColor',[0.10 0.35 0.90], 'EdgeColor','none', ...
    'FaceAlpha',0.98, 'HandleVisibility','off');

attach_sphere = surf(ax, nan(size(sx)), nan(size(sy)), nan(size(sz)), ...
    'FaceColor',[0.90 0.10 0.10], 'EdgeColor','none', ...
    'FaceAlpha',0.98, 'HandleVisibility','off');

light(ax, 'Position',[1 -2 3], 'Style','infinite');
lighting(ax, 'gouraud');
material(ax, 'dull');

legend(ax, 'Location','northeastoutside');

time_text = annotation(fig, 'textbox', [0.13 0.90 0.18 0.045], ...
    'String', '', ...
    'FontSize', 13, ...
    'FontWeight', 'bold', ...
    'EdgeColor', 'none', ...
    'BackgroundColor', 'none', ...
    'Color', [0 0 0], ...
    'Interpreter', 'none');

apply_clean_axis_ticks(fig);

% =========================================================
% MAIN VIDEO WRITER
% =========================================================

if save_video
    try
        vw = VideoWriter(video_mp4, 'MPEG-4');
        video_name = video_mp4;
    catch
        vw = VideoWriter(video_avi);
        video_name = video_avi;
    end

    vw.FrameRate = frame_rate;

    try
        vw.Quality = video_quality;
    catch
    end

    open(vw);
else
    vw = [];
    video_name = '';
end

% =========================================================
% SYNCHRONIZED PLOT VIDEO WRITERS
% =========================================================

plot_figs = gobjects(0,1);
plot_cursor_lines = {};
plot_vw = {};
plot_video_files = {};

if save_plot_videos
    n_plot_videos = numel(plot_video_names);

    plot_figs = gobjects(n_plot_videos,1);
    plot_cursor_lines = cell(n_plot_videos,1);
    plot_vw = cell(n_plot_videos,1);
    plot_video_files = cell(n_plot_videos,1);

    for pp = 1:n_plot_videos

        this_plot_name = plot_video_names{pp};

        [plot_figs(pp), plot_cursor_lines{pp}] = create_synced_plot_figure( ...
            plot_data, ...
            this_plot_name, ...
            T_start, ...
            T_end, ...
            plot_cursor_color, ...
            plot_cursor_width, ...
            plot_cursor_style, ...
            plot_video_position + [40*(pp-1), 40*(pp-1), 0, 0]);

        plot_video_files{pp} = fullfile(save_dir, ...
            [video_basename '_' this_plot_name '.mp4']);

        try
            plot_vw{pp} = VideoWriter(plot_video_files{pp}, 'MPEG-4');
        catch
            plot_video_files{pp} = fullfile(save_dir, ...
                [video_basename '_' this_plot_name '.avi']);
            plot_vw{pp} = VideoWriter(plot_video_files{pp});
        end

        plot_vw{pp}.FrameRate = frame_rate;

        try
            plot_vw{pp}.Quality = video_quality;
        catch
        end

        open(plot_vw{pp});
    end
end

% =========================================================
% ANIMATION LOOP
% =========================================================

for jj = 1:numel(frame_ids)
    k = frame_ids(jj);

    xL = xL_all(k,:).';
    q = q_all(k,:).';
    R = R_all(:,:,k);

    % Recompute the rotated pendulum/drone geometry from the current state.
    if use_state_derived_geometry
        xP = xL - L*q;
        xQ = xP - R*r;
    else
        xP = xP_all(k,:).';
        xQ = xQ_all(k,:).';
    end

    b1 = R(:,1);
    b2 = R(:,2);
    b3 = R(:,3);

    % Visual-only yaw rotation of the quadrotor arm cross about body z-axis b3.
    R_arm = R * rotz_local(arm_yaw_visual);
    b1_arm = R_arm(:,1);
    b2_arm = R_arm(:,2);

    a1 = xQ + R_arm*[ arm_len; 0; 0];
    a2 = xQ + R_arm*[-arm_len; 0; 0];
    a3 = xQ + R_arm*[0;  arm_len; 0];
    a4 = xQ + R_arm*[0; -arm_len; 0];

    if isfinite(trail_samples)
        k0 = max(frame_ids(1), k-trail_samples+1);
    else
        k0 = frame_ids(1);
    end

    set(load_trail, ...
        'XData', xL_v(k0:k,1), ...
        'YData', xL_v(k0:k,2), ...
        'ZData', xL_v(k0:k,3));

    set(com_trail, ...
        'XData', xQ_v(k0:k,1), ...
        'YData', xQ_v(k0:k,2), ...
        'ZData', xQ_v(k0:k,3));

    set_line3(cable_line, xP, xL, z_invert_for_display);
    set_line3(offset_line, xQ, xP, z_invert_for_display);
    set_line3(arm1_line, a1, a2, z_invert_for_display);
    set_line3(arm2_line, a3, a4, z_invert_for_display);

    centers = [a1, a2, a3, a4];

    for i = 1:4
        C = draw_circle3(centers(:,i), b1_arm, b2_arm, prop_radius);
        Cv = viz_points(C.', z_invert_for_display).';
        set(prop_lines(i), ...
            'XData', Cv(1,:), ...
            'YData', Cv(2,:), ...
            'ZData', Cv(3,:));
    end

    if show_body_axes
        set_line3(body_x_line, xQ, xQ + body_axis_len*b1, z_invert_for_display);
        set_line3(body_y_line, xQ, xQ + body_axis_len*b2, z_invert_for_display);
        set_line3(body_z_line, xQ, xQ + body_axis_len*b3, z_invert_for_display);
    else
        hide_line3(body_x_line);
        hide_line3(body_y_line);
        hide_line3(body_z_line);
    end

    update_sphere(load_sphere, sx, sy, sz, xL, load_radius, z_invert_for_display);
    update_sphere(com_sphere, sx, sy, sz, xQ, com_radius, z_invert_for_display);
    update_sphere(attach_sphere, sx, sy, sz, xP, attach_radius, z_invert_for_display);

    time_text.String = sprintf('t = %.2f s', T(k));

    drawnow;

    % Write the main 3D animation frame.
    if save_video
        frame = getframe(fig);
        frame = make_even_video_frame(frame);
        writeVideo(vw, frame);
    end

    % Write the synchronized plot-video frames.
    if save_plot_videos
        for pp = 1:numel(plot_vw)

            update_plot_time_cursor(plot_cursor_lines{pp}, T(k));

            drawnow;

            plot_frame = getframe(plot_figs(pp));
            plot_frame = make_even_video_frame(plot_frame);

            writeVideo(plot_vw{pp}, plot_frame);
        end
    end

    if mod(jj, 100) == 0
        fprintf('Frame %d / %d\n', jj, numel(frame_ids));
    end
end

% =========================================================
% SAVE OUTPUTS
% =========================================================

if save_video
    close(vw);
    fprintf('\nSaved animation video to:\n  %s\n', video_name);
end

if save_plot_videos
    for pp = 1:numel(plot_vw)
        close(plot_vw{pp});
        fprintf('Saved synchronized plot video to:\n  %s\n', plot_video_files{pp});
    end
end

if save_final_frame
    final_png = fullfile(save_dir, [video_basename '_final_frame.png']);
    final_fig = fullfile(save_dir, [video_basename '_final_frame.fig']);

    saveas(fig, final_png);
    savefig(fig, final_fig);

    if exist('saveFigureAsPDF', 'file') == 2
        try
            saveFigureAsPDF(fig, fullfile(save_dir, [video_basename '_final_frame']), 10, 7, 600);
        catch ME
            warning('Could not save final-frame PDF: %s', ME.message);
        end
    end

    fprintf('Saved final frame to:\n  %s\n', final_png);
end

if close_after_save
    if isvalid(fig)
        close(fig);
    end

    if save_plot_videos
        for pp = 1:numel(plot_figs)
            if isvalid(plot_figs(pp))
                close(plot_figs(pp));
            end
        end
    end
end

fprintf('\nAnimation output directory:\n  %s\n\n', save_dir);

% =========================================================
% LOCAL FUNCTIONS
% =========================================================

function [xL, q, R] = unpack_full_state(x)

xL = x(1:3);

q = x(4:6);
q = q / norm(q);

R = [
    x(7),  x(8),  x(9);
    x(10), x(11), x(12);
    x(13), x(14), x(15)
];

end

function plot_data = build_plot_data(sim, T, xL_all, ref_all)

n = numel(T);

plot_data.T = T;

% Position tracking error.
% Prefer simulator-stored sim.err if available.
% Otherwise compute actual load position minus reference load position.
if isfield(sim, 'err')
    e_pos = normalize_signal_matrix(sim.err, n, 3);
    if ~all(isfinite(e_pos(:)))
        e_pos = xL_all(:,1:3) - ref_all(:,1:3);
    end
else
    e_pos = xL_all(:,1:3) - ref_all(:,1:3);
end

plot_data.e_pos = e_pos;
plot_data.e_pos_norm = sqrt(sum(e_pos.^2, 2));

% Control inputs.
plot_data.u_t = get_sim_signal(sim, {'u_t','ut','thrust','f'}, n, 1);
plot_data.tau_cm = get_sim_signal(sim, {'tau_cm','tauCM','tau_c_m','taucm'}, n, 3);
plot_data.tau_b  = get_sim_signal(sim, {'tau_b','taub','tau_body','tau'}, n, 3);

% Attitude errors.
plot_data.eR = get_sim_signal(sim, {'eR','e_R','err_R','attitude_error'}, n, 3);
plot_data.eOmega = get_sim_signal(sim, {'eOmega','e_Omega','eW','e_omega','omega_error'}, n, 3);

plot_data.eR_norm = vector_norm_or_nan(plot_data.eR);
plot_data.eOmega_norm = vector_norm_or_nan(plot_data.eOmega);

end

function Y = get_sim_signal(sim, candidate_names, n, ncols)

Y = nan(n, ncols);

for i = 1:numel(candidate_names)
    name = candidate_names{i};

    if isfield(sim, name)
        temp = normalize_signal_matrix(sim.(name), n, ncols);

        if any(isfinite(temp(:)))
            Y = temp;
            return;
        end
    end
end

end

function Y = normalize_signal_matrix(V, n, ncols)

Y = nan(n, ncols);

if isempty(V)
    return;
end

if isvector(V)
    V = V(:);

    if numel(V) == n && ncols == 1
        Y(:,1) = V;
    end

    return;
end

if size(V,1) == n
    cols = min(ncols, size(V,2));
    Y(:,1:cols) = V(:,1:cols);
    return;
end

if size(V,2) == n
    rows = min(ncols, size(V,1));
    Y(:,1:rows) = V(1:rows,:).';
    return;
end

end

function y = vector_norm_or_nan(Y)

if any(isfinite(Y(:)))
    y = sqrt(sum(Y.^2, 2));
else
    y = nan(size(Y,1), 1);
end

end

function tf = signal_available(Y)

tf = any(isfinite(Y(:)));

end

function Pviz = viz_points(P, invert_z)

Pviz = P;

if invert_z
    Pviz(:,3) = -Pviz(:,3);
end

end

function pviz = viz_point(p, invert_z)

pviz = p(:);

if invert_z
    pviz(3) = -pviz(3);
end

end

function set_line3(h, p1, p2, invert_z)

p1v = viz_point(p1, invert_z);
p2v = viz_point(p2, invert_z);

set(h, ...
    'XData', [p1v(1), p2v(1)], ...
    'YData', [p1v(2), p2v(2)], ...
    'ZData', [p1v(3), p2v(3)]);

end

function update_sphere(h, sx, sy, sz, center, radius, invert_z)

C = viz_point(center, invert_z);

set(h, ...
    'XData', C(1) + radius*sx, ...
    'YData', C(2) + radius*sy, ...
    'ZData', C(3) + radius*sz);

end

function C = draw_circle3(center, e1, e2, radius)

theta = linspace(0, 2*pi, 80);
C = center + radius*(e1*cos(theta) + e2*sin(theta));

end

function Rz = rotz_local(theta)

c = cos(theta);
s = sin(theta);

Rz = [
    c, -s, 0;
    s,  c, 0;
    0,  0, 1
];

end

function hide_line3(h)

set(h, 'XData', nan, 'YData', nan, 'ZData', nan);

end

function lims = compute_equal_lims(P, pad_fraction)

mins = min(P, [], 1);
maxs = max(P, [], 1);

center = 0.5*(mins + maxs);
span = max(maxs - mins);

if span < 1e-6
    span = 1;
end

pad = pad_fraction*span;
half_width = 0.5*span + pad;

lims = [
    center(1)-half_width, center(1)+half_width;
    center(2)-half_width, center(2)+half_width;
    center(3)-half_width, center(3)+half_width
];

end

function [fig, cursor_lines] = create_synced_plot_figure( ...
    plot_data, plot_name, T_start, T_end, cursor_color, cursor_width, cursor_style, fig_position)

T = plot_data.T;

fig = figure( ...
    'Color','w', ...
    'Position', fig_position, ...
    'Name', [plot_name '_synced_video']);

cursor_lines = gobjects(0,1);

switch lower(plot_name)

    case 'errors_position'

        tiledlayout(4,1,'TileSpacing','compact','Padding','compact');

        for i = 1:3
            ax = nexttile;

            if signal_available(plot_data.e_pos(:,i))
                plot(ax, T, plot_data.e_pos(:,i), 'LineWidth', 1.4);
            else
                missing_axis_message(ax, 'position error not available');
            end

            grid(ax, 'on');
            ylabel(ax, sprintf('$e_{x,%d}$', i), 'Interpreter','latex');

            prepare_synced_axis(ax, T_start, T_end);
            cursor_lines(end+1,1) = add_time_cursor( ...
                ax, T_start, cursor_color, cursor_width, cursor_style);
        end

        ax = nexttile;

        if signal_available(plot_data.e_pos_norm)
            plot(ax, T, plot_data.e_pos_norm, 'LineWidth', 1.5);
        else
            missing_axis_message(ax, 'position error norm not available');
        end

        grid(ax, 'on');
        ylabel(ax, '$\|e_x\|_2$', 'Interpreter','latex');
        xlabel(ax, '$t~[\mathrm{s}]$', 'Interpreter','latex');

        prepare_synced_axis(ax, T_start, T_end);
        cursor_lines(end+1,1) = add_time_cursor( ...
            ax, T_start, cursor_color, cursor_width, cursor_style);

        sgtitle(fig, 'Position Tracking Error', 'Interpreter','latex');


    case 'inputs'

        tiledlayout(4,1,'TileSpacing','compact','Padding','compact');

        ax = nexttile;

        if signal_available(plot_data.u_t)
            plot(ax, T, plot_data.u_t, 'LineWidth', 1.4);
        else
            missing_axis_message(ax, 'sim.u_t not found');
        end

        grid(ax, 'on');
        ylabel(ax, '$u_t$', 'Interpreter','latex');

        prepare_synced_axis(ax, T_start, T_end);
        cursor_lines(end+1,1) = add_time_cursor( ...
            ax, T_start, cursor_color, cursor_width, cursor_style);

        for i = 1:3
            ax = nexttile;
            hold(ax, 'on');

            has_cm = signal_available(plot_data.tau_cm(:,i));
            has_b  = signal_available(plot_data.tau_b(:,i));

            if has_cm
                plot(ax, T, plot_data.tau_cm(:,i), 'LineWidth', 1.3, ...
                    'DisplayName','$\tau_{\mathrm{cm}}$');
            end

            if has_b
                plot(ax, T, plot_data.tau_b(:,i), '--', 'LineWidth', 1.1, ...
                    'DisplayName','$\tau_b$');
            end

            if ~has_cm && ~has_b
                missing_axis_message(ax, 'torque signal not found');
            end

            grid(ax, 'on');
            ylabel(ax, sprintf('$\\tau_%d$', i), 'Interpreter','latex');

            if i == 1 && (has_cm || has_b)
                legend(ax, 'Interpreter','latex','Location','best');
            end

            if i == 3
                xlabel(ax, '$t~[\mathrm{s}]$', 'Interpreter','latex');
            end

            prepare_synced_axis(ax, T_start, T_end);
            cursor_lines(end+1,1) = add_time_cursor( ...
                ax, T_start, cursor_color, cursor_width, cursor_style);
        end

        sgtitle(fig, 'Control Inputs', 'Interpreter','latex');


    case 'attitude_errors'

        tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

        ax = nexttile;

        if signal_available(plot_data.eR)
            plot(ax, T, plot_data.eR, 'LineWidth', 1.2);
            legend(ax, {'$e_{R1}$','$e_{R2}$','$e_{R3}$'}, ...
                'Interpreter','latex','Location','best');
        else
            missing_axis_message(ax, 'sim.eR not found');
        end

        grid(ax, 'on');
        ylabel(ax, '$e_R$', 'Interpreter','latex');

        prepare_synced_axis(ax, T_start, T_end);
        cursor_lines(end+1,1) = add_time_cursor( ...
            ax, T_start, cursor_color, cursor_width, cursor_style);

        ax = nexttile;

        if signal_available(plot_data.eR_norm)
            plot(ax, T, plot_data.eR_norm, 'LineWidth', 1.4);
        else
            missing_axis_message(ax, 'attitude error norm not available');
        end

        grid(ax, 'on');
        ylabel(ax, '$\|e_R\|_2$', 'Interpreter','latex');

        prepare_synced_axis(ax, T_start, T_end);
        cursor_lines(end+1,1) = add_time_cursor( ...
            ax, T_start, cursor_color, cursor_width, cursor_style);

        ax = nexttile;

        if signal_available(plot_data.eOmega_norm)
            plot(ax, T, plot_data.eOmega_norm, 'LineWidth', 1.4);
        else
            missing_axis_message(ax, 'sim.eOmega not found');
        end

        grid(ax, 'on');
        ylabel(ax, '$\|e_\Omega\|_2$', 'Interpreter','latex');
        xlabel(ax, '$t~[\mathrm{s}]$', 'Interpreter','latex');

        prepare_synced_axis(ax, T_start, T_end);
        cursor_lines(end+1,1) = add_time_cursor( ...
            ax, T_start, cursor_color, cursor_width, cursor_style);

        sgtitle(fig, 'Attitude Tracking Errors', 'Interpreter','latex');


    otherwise
        error('Unknown synchronized plot video name: %s', plot_name);
end

apply_clean_axis_ticks(fig);

end

function prepare_synced_axis(ax, T_start, T_end)

xlim(ax, [T_start, T_end]);
box(ax, 'on');

yl = ylim(ax);

if ~all(isfinite(yl)) || abs(yl(2)-yl(1)) < 1e-9
    ylim(ax, [-1 1]);
else
    pad = 0.05*(yl(2)-yl(1));
    ylim(ax, [yl(1)-pad, yl(2)+pad]);
end

end

function h = add_time_cursor(ax, t_now, cursor_color, cursor_width, cursor_style)

yl = ylim(ax);

h = line(ax, ...
    [t_now, t_now], yl, ...
    'Color', cursor_color, ...
    'LineWidth', cursor_width, ...
    'LineStyle', cursor_style, ...
    'HandleVisibility', 'off');

try
    uistack(h, 'top');
catch
end

end

function update_plot_time_cursor(cursor_lines, t_now)

for i = 1:numel(cursor_lines)
    if ~isvalid(cursor_lines(i))
        continue;
    end

    ax = ancestor(cursor_lines(i), 'axes');
    yl = ylim(ax);

    set(cursor_lines(i), ...
        'XData', [t_now, t_now], ...
        'YData', yl);

    try
        uistack(cursor_lines(i), 'top');
    catch
    end
end

end

function missing_axis_message(ax, msg)

text(ax, 0.5, 0.5, msg, ...
    'Units','normalized', ...
    'HorizontalAlignment','center', ...
    'VerticalAlignment','middle', ...
    'FontSize', 11, ...
    'Color', [0.45 0.45 0.45], ...
    'Interpreter','none');

ylim(ax, [0 1]);

end

function apply_clean_axis_ticks(fig_handle)

if nargin < 1 || isempty(fig_handle)
    fig_handle = gcf;
end

axs = findall(fig_handle, 'Type', 'axes');

for ii = 1:numel(axs)
    ax = axs(ii);

    try
        xtickformat(ax, '%.2f');
        ytickformat(ax, '%.2f');
        ztickformat(ax, '%.2f');
    catch
        try
            ax.XAxis.TickLabelFormat = '%.2f';
            ax.YAxis.TickLabelFormat = '%.2f';

            if isprop(ax, 'ZAxis')
                ax.ZAxis.TickLabelFormat = '%.2f';
            end
        catch
        end
    end
end

end

function frame = make_even_video_frame(frame)

[h, w, ~] = size(frame.cdata);

if mod(w,2) ~= 0
    frame.cdata(:,end+1,:) = frame.cdata(:,end,:);
end

if mod(h,2) ~= 0
    frame.cdata(end+1,:,:) = frame.cdata(end,:,:);
end

end