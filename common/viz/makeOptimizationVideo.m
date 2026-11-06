function makeOptimizationVideo(model, result, videoFile, opts)
% MAKEOPTIMIZATIONVIDEO  How STOMP reshapes the trajectory, one frame per iteration.
% Left: the end-effector path of the current iteration (black) over the straight-line
% initial guess (dotted) and the previous iterations (fading grey), with the robot at
% the waypoint that is closest to an obstacle in the INITIAL guess. Right: total cost
% Q(theta) up to the current iteration.
%
% INPUT  result    : runPipeline output (needs .theta_animation, .Q_time, .world, .scene)
%        videoFile : output .mp4
%        opts      : (optional) .frameRate (default 8), .holdSec (default 1.5), .title

if nargin < 4, opts = struct(); end
frameRate = getOpt(opts, 'frameRate', 8);
holdSec   = getOpt(opts, 'holdSec', 1.5);
figTitle  = getOpt(opts, 'title', '');

TH = result.theta_animation;                    % {1} = init, {k+1} = after iteration k
nIt = numel(TH) - 1;
N = size(TH{1}, 2);
% waypoint to show the robot at: the most colliding one of the initial guess
flags = checkTrajCollision(model.robot, TH{1}, result.world);
if any(flags), wShow = round(median(find(flags))); else, wShow = round(N / 2); end

fig = figure('Name', 'optim_video', 'Color', 'w', 'Position', [50 50 1240 540]);
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
if ~isempty(figTitle), title(tl, figTitle, 'FontWeight', 'bold'); end
ax = nexttile(tl);  hold(ax, 'on');
for i = 1:numel(result.world)
    [~, p] = show(result.world{i}, 'Parent', ax);
    set(p, 'LineStyle', 'none', 'FaceColor', [0.85 0.20 0.15], 'FaceAlpha', 0.45);
end
P0 = eePositions(model, TH{1});
plot3(ax, P0(1, :), P0(2, :), P0(3, :), ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5);
plot3(ax, P0(1, 1), P0(2, 1), P0(3, 1), 'o', 'MarkerSize', 9, 'MarkerFaceColor', [0.20 0.70 0.25], 'MarkerEdgeColor', 'k');
plot3(ax, P0(1, end), P0(2, end), P0(3, end), 's', 'MarkerSize', 9, 'MarkerFaceColor', [0.90 0.15 0.15], 'MarkerEdgeColor', 'k');
hOld = gobjects(0);
hCur = plot3(ax, NaN, NaN, NaN, '-o', 'Color', [0.05 0.05 0.05], 'LineWidth', 2.2, 'MarkerSize', 3);
axis(ax, result.scene.axisLimits);
view(ax, result.scene.viewAngle(1), result.scene.viewAngle(2));
grid(ax, 'on');  xlabel(ax, 'x [m]');  ylabel(ax, 'y [m]');  zlabel(ax, 'z [m]');

axC = nexttile(tl);  hold(axC, 'on');  grid(axC, 'on');
Qc = result.Q_time;
hQ = plot(axC, NaN, NaN, '-', 'Color', [0.00 0.45 0.74], 'LineWidth', 2);
xlim(axC, [0, max(nIt, 1)]);
if all(Qc > 0) && max(Qc) / min(Qc) > 20, set(axC, 'YScale', 'log'); end
xlabel(axC, 'Iteration');  ylabel(axC, 'Total cost Q(\theta)');

v = VideoWriter(videoFile, 'MPEG-4');
v.FrameRate = frameRate;  v.Quality = 95;
open(v);
nHold = round(holdSec * frameRate);
for k = 0:nIt
    th = TH{k + 1};
    P = eePositions(model, th);
    if k > 0                                                   % previous path fades out
        hOld(end + 1) = copyobj(hCur, ax); %#ok<AGROW>
        set(hOld(end), 'Color', [0.75 0.75 0.75], 'LineWidth', 0.8, 'Marker', 'none');
    end
    set(hCur, 'XData', P(1, :), 'YData', P(2, :), 'ZData', P(3, :));
    uistack(hCur, 'top');
    show(model.robot, th(:, wShow), 'Parent', ax, 'PreservePlot', false, 'FastUpdate', true, 'Frames', 'off');
    axis(ax, result.scene.axisLimits);
    collides = any(checkTrajCollision(model.robot, th, result.world));
    title(ax, sprintf('iteration %d / %d  --  %s', k, nIt, ...
        ternary(collides, 'path collides', 'collision-free')), 'Color', ternary(collides, [0.8 0.1 0.1], [0.1 0.5 0.1]));
    if k > 0, set(hQ, 'XData', 1:k, 'YData', Qc(1:k)); end
    drawnow;
    frame = getframe(fig);
    reps = 1 + nHold * (k == 0 || k == nIt);
    for i = 1:reps, writeVideo(v, frame); end
end
close(v);
close(fig);
fprintf('makeOptimizationVideo: %s (%d iterations)\n', videoFile, nIt);
end


function P = eePositions(model, Q)
P = zeros(3, size(Q, 2));
for t = 1:size(Q, 2)
    T = getTransform(model.robot, Q(:, t), model.eeName);
    P(:, t) = T(1:3, 4);
end
end


function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end


function v = getOpt(s, name, default)
if isfield(s, name) && ~isempty(s.(name)), v = s.(name); else, v = default; end
end
