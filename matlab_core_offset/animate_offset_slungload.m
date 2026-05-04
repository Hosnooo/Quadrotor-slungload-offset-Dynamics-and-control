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

trajectory_name = 'regulation_steps';

results_root = fullfile(pwd, 'results_offset_fulldynamics');
mat_name = fullfile(results_root, trajectory_name, 'sim_offset_QSFA_U.mat');

video_basename = 'offset_slung_load_animation';

% Video settings
T_start = 0;
T_end   = 40;

frame_rate = 30;          % smooth video
playback_speed = 3;       % simulation seconds per video second
video_quality = 95;       % MP4 quality

trail_seconds = inf;      % inf = full trail
z_invert_for_display = true;

save_video = true;
save_final_frame = true;
close_after_save = false;

% Visual scale settings
arm_len = 0.18;
prop_radius = 0.045;
load_radius = 0.03;
com_radius = 0.035;
attach_radius = 0.027;

% Visual-only rotation of quad arms about body z-axis b3.
% This does not change the true attitude, CoM, suspension point, or offset arm.
arm_yaw_visual = deg2rad(35);

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

if isfield(sim, 'xP')
    xP_all = sim.xP;
else
    xP_all = zeros(n,3);
end

if isfield(sim, 'xQ')
    xQ_all = sim.xQ;
else
    xQ_all = zeros(n,3);
end

R_all = zeros(3,3,n);
q_all = zeros(n,3);

for k = 1:n
    [xL, q, R] = unpack_full_state(X(k,:).');
    q_all(k,:) = q.';
    R_all(:,:,k) = R;

    if ~isfield(sim, 'xP')
        xP = xL - L*q;
        xP_all(k,:) = xP.';
    end

    if ~isfield(sim, 'xQ')
        xP = xP_all(k,:).';
        xQ = xP - R*r;
        xQ_all(k,:) = xQ.';
    end
end

if isfield(sim, 'ref')
    ref_all = sim.ref;
else
    ref_all = xL_all;
end

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
fprintf('Number of animation frames: %d\n', numel(frame_ids));
fprintf('Approximate output video duration: %.2f s\n', numel(frame_ids)/frame_rate);

% =========================================================
% FIGURE SETUP
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
% VIDEO WRITER
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
% ANIMATION LOOP
% =========================================================

for jj = 1:numel(frame_ids)
    k = frame_ids(jj);

    xL = xL_all(k,:).';
    xP = xP_all(k,:).';
    xQ = xQ_all(k,:).';
    R = R_all(:,:,k);

    b1 = R(:,1);
    b2 = R(:,2);
    b3 = R(:,3);

    % Visual-only yaw rotation of the quadrotor arm cross about body z-axis b3.
    % The physical CoM, suspension point, cable, and offset arm are unchanged.
    b1_arm =  cos(arm_yaw_visual)*b1 + sin(arm_yaw_visual)*b2;
    b2_arm = -sin(arm_yaw_visual)*b1 + cos(arm_yaw_visual)*b2;

    a1 = xQ + arm_len*b1_arm;
    a2 = xQ - arm_len*b1_arm;
    a3 = xQ + arm_len*b2_arm;
    a4 = xQ - arm_len*b2_arm;

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

    update_sphere(load_sphere, sx, sy, sz, xL, load_radius, z_invert_for_display);
    update_sphere(com_sphere, sx, sy, sz, xQ, com_radius, z_invert_for_display);
    update_sphere(attach_sphere, sx, sy, sz, xP, attach_radius, z_invert_for_display);

    time_text.String = sprintf('t = %.2f s', T(k));

    drawnow;

    if save_video
        frame = getframe(fig);
        frame = make_even_video_frame(frame);
        writeVideo(vw, frame);
    end
end

% =========================================================
% SAVE OUTPUTS
% =========================================================

if save_video
    close(vw);
    fprintf('\nSaved animation video to:\n  %s\n', video_name);
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
    close(fig);
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
            ax.ZAxis.TickLabelFormat = '%.2f';
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