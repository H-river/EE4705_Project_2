function makeTrajectoryVideo(model, runs, labels, videoFile, opts)
% MAKETRAJECTORYVIDEO  Presentation video (MPEG-4) of one or more planned trajectories.
% Several runs of the same robot are played side by side and in sync (e.g. Task 5
% without / with the constraint). Compared with animateTrajectory (one frame per
% waypoint, 2 fps), the motion between waypoints is interpolated in joint space -- the
% same interpolation checkTrajCollisionDense verifies -- so the video is smooth.
%
% INPUT  runs      : result struct or 1 x K cell of result structs (runPipeline output,
%                    needs .theta, .thetaInit, .world, .scene)
%        labels    : 1 x K cell of tile titles
%        videoFile : output file (.mp4)
%        opts      : (optional) .subSteps   frames per waypoint segment (default 10)
%                               .frameRate  (default 24)
%                               .holdSec    still frames at start and end [s] (default 1)
%                               .showEEAxis draw the EE y-axis (Task 5)  (default false)
%                               .title      overall title (default '')

if ~iscell(runs), runs = {runs}; end
if nargin < 3 || isempty(labels), labels = repmat({''}, 1, numel(runs)); end
if nargin < 5, opts = struct(); end
subSteps   = getOpt(opts, 'subSteps', 10);
frameRate  = getOpt(opts, 'frameRate', 24);
holdSec    = getOpt(opts, 'holdSec', 1);
showEEAxis = getOpt(opts, 'showEEAxis', false);
figTitle   = getOpt(opts, 'title', '');

K = numel(runs);
fig = figure('Name', 'video', 'Color', 'w', 'Position', [50 50 620 * K 560]);
tl = tiledlayout(fig, 1, K, 'TileSpacing', 'compact', 'Padding', 'compact');
if ~isempty(figTitle), title(tl, figTitle, 'FontWeight', 'bold'); end

ax = gobjects(1, K);  trail = gobjects(1, K);  arrow = gobjects(1, K);
Q = cell(1, K);  P = cell(1, K);
for k = 1:K
    r = runs{k};
    Q{k} = densify(r.theta, subSteps);
    P{k} = eePositions(model, Q{k});
    ax(k) = nexttile(tl);  hold(ax(k), 'on');
    for i = 1:numel(r.world)                                   % obstacles
        [~, p] = show(r.world{i}, 'Parent', ax(k));
        set(p, 'LineStyle', 'none', 'FaceColor', [0.85 0.20 0.15], 'FaceAlpha', 0.45);
    end
    Pinit = eePositions(model, r.thetaInit);                   % straight-line initial guess
    plot3(ax(k), Pinit(1, :), Pinit(2, :), Pinit(3, :), ':', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.2);
    plot3(ax(k), P{k}(1, 1), P{k}(2, 1), P{k}(3, 1), 'o', 'MarkerSize', 9, ...
        'MarkerFaceColor', [0.20 0.70 0.25], 'MarkerEdgeColor', 'k');
    plot3(ax(k), P{k}(1, end), P{k}(2, end), P{k}(3, end), 's', 'MarkerSize', 9, ...
        'MarkerFaceColor', [0.90 0.15 0.15], 'MarkerEdgeColor', 'k');
    trail(k) = plot3(ax(k), NaN, NaN, NaN, '-', 'Color', [0.1 0.1 0.1], 'LineWidth', 2);
    axis(ax(k), r.scene.axisLimits);
    view(ax(k), r.scene.viewAngle(1), r.scene.viewAngle(2));
    grid(ax(k), 'on');  xlabel(ax(k), 'x [m]');  ylabel(ax(k), 'y [m]');  zlabel(ax(k), 'z [m]');
    title(ax(k), labels{k}, 'Interpreter', 'none');
end

v = VideoWriter(videoFile, 'MPEG-4');
v.FrameRate = frameRate;  v.Quality = 95;
open(v);
nF = size(Q{1}, 2);
nHold = round(holdSec * frameRate);
for f = 1:nF
    for k = 1:K
        q = Q{k}(:, min(f, size(Q{k}, 2)));
        show(model.robot, q, 'Parent', ax(k), 'PreservePlot', false, 'FastUpdate', true, 'Frames', 'off');
        set(trail(k), 'XData', P{k}(1, 1:f), 'YData', P{k}(2, 1:f), 'ZData', P{k}(3, 1:f));
        if showEEAxis
            T = getTransform(model.robot, q, model.eeName);
            a = 0.12 * T(1:3, 2);  p = T(1:3, 4);
            if isgraphics(arrow(k)), delete(arrow(k)); end
            arrow(k) = quiver3(ax(k), p(1), p(2), p(3), a(1), a(2), a(3), 0, ...
                'Color', [0.93 0.55 0.05], 'LineWidth', 3, 'MaxHeadSize', 0.8);
        end
        axis(ax(k), runs{k}.scene.axisLimits);
    end
    drawnow;
    frame = getframe(fig);
    reps = 1 + nHold * (f == 1 || f == nF);                    % hold first / last frame
    for i = 1:reps, writeVideo(v, frame); end
end
close(v);
close(fig);
fprintf('makeTrajectoryVideo: %s (%d frames)\n', videoFile, nF + 2 * nHold);
end


function Qd = densify(theta, nSub)
% Joint-space linear interpolation, nSub frames per segment (+ the last waypoint)
N = size(theta, 2);
Qd = zeros(size(theta, 1), (N - 1) * nSub + 1);
a = (0:nSub - 1) / nSub;
for i = 1:N - 1
    Qd(:, (i - 1) * nSub + (1:nSub)) = theta(:, i) + (theta(:, i + 1) - theta(:, i)) .* a;
end
Qd(:, end) = theta(:, end);
end


function P = eePositions(model, Q)
P = zeros(3, size(Q, 2));
for t = 1:size(Q, 2)
    T = getTransform(model.robot, Q(:, t), model.eeName);
    P(:, t) = T(1:3, 4);
end
end


function v = getOpt(s, name, default)
if isfield(s, name) && ~isempty(s.(name)), v = s.(name); else, v = default; end
end
