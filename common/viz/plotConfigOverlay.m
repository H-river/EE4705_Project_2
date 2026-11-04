function fig = plotConfigOverlay(model, theta, world, scene, opts)
% PLOTCONFIGOVERLAY  Static figure for report/slides: intermediate configurations of
% a trajectory overlaid in one plot (handout Task 4: "overlapping the intermediate
% configurations"). Used by Tasks 2-5 -> keep the call signature stable.
%
% INPUT  theta : nJoints x nDiscretize
%        world : collision objects (from buildVoxelWorld)
%        scene : for axisLimits / viewAngle / name
%        opts  : (optional) struct, suggested fields
%                .stride   plot every k-th waypoint (default 2)
%                .showEEPath  draw the EE path as a line (default true)
%                .title, .saveFile ('' = do not save; e.g. 'fig.png')
%
% Extra optional fields (all have defaults, so existing calls keep working):
%                .showEEAxis  draw the constrained EE body axis at every plotted
%                             waypoint as a short arrow (default false; useful in Task 5)
%                .eeAxis      which EE body axis to draw, 1/2/3 (default 2 = y)
%                .axisLen     arrow length [m] (default 0.08)
%                .colorStart, .colorGoal  1x3 RGB of the first / last configuration
%                .alphaStart, .alphaGoal  face alpha of the first / last configuration
%
% Only uses Robotics System Toolbox (show, getTransform, tform2trvec, tform2rotm) and
% base MATLAB -- it does NOT depend on STOMP, PoE or any Task 1-4 code, so it can be
% tested on a plain straight-line trajectory.
%
% Owner: T5

if nargin < 5 || isempty(opts), opts = struct(); end

%% Options
stride      = getOpt(opts, 'stride',      2);
showEEPath  = getOpt(opts, 'showEEPath',  true);
showEEAxis  = getOpt(opts, 'showEEAxis',  false);
eeAxis      = getOpt(opts, 'eeAxis',      2);
axisLen     = getOpt(opts, 'axisLen',     0.08);
colorStart  = getOpt(opts, 'colorStart',  [0.72 0.80 0.92]);   % light blue-grey
colorGoal   = getOpt(opts, 'colorGoal',   [0.05 0.25 0.65]);   % deep blue
alphaStart  = getOpt(opts, 'alphaStart',  0.15);
alphaGoal   = getOpt(opts, 'alphaGoal',   0.75);
figTitle    = getOpt(opts, 'title',       scene.name);
saveFile    = getOpt(opts, 'saveFile',    '');

N   = size(theta, 2);
idx = unique([1:stride:N, N]);          % always include the start and the goal
nIdx = numel(idx);

%% Figure and axes
fig = figure('Name', ['overlay_' scene.name], 'Position', [100 100 700 560]);
ax  = axes('Parent', fig);
hold(ax, 'on');

%% Obstacles (drawn first, so they are not touched by the fading code below)
hObs = [];
for i = 1:numel(world)
    [~, pObj] = show(world{i}, 'Parent', ax);
    pObj.LineStyle = 'none';
    pObj.FaceColor = [0.85 0.20 0.15];
    pObj.FaceAlpha = 0.45;
    if i == 1, hObs = pObj; end
end

%% Robot configurations, faded from start (light, transparent) to goal (dark)
for j = 1:nIdx
    s = (j - 1) / max(nIdx - 1, 1);                 % 0 at start, 1 at goal
    col = (1 - s) * colorStart + s * colorGoal;
    alp = (1 - s) * alphaStart + s * alphaGoal;

    before = ax.Children;
    show(model.robot, theta(:, idx(j)), 'Parent', ax, ...
         'PreservePlot', true, 'Frames', 'off', 'FastUpdate', false);
    hold(ax, 'on');
    after = ax.Children;

    % graphics objects that this show() call added
    added = after(~ismember(after, before));
    patches = findobj(added, 'Type', 'patch');
    set(patches, 'FaceColor', col, 'FaceAlpha', alp, 'EdgeColor', 'none');
end

%% End-effector path (and optionally the constrained EE axis)
P = zeros(3, N);
A = zeros(3, N);
for t = 1:N
    T = getTransform(model.robot, theta(:, t), model.eeName);
    P(:, t) = tform2trvec(T)';
    R = tform2rotm(T);
    A(:, t) = R(:, eeAxis);                 % EE body axis in the world frame
end

hPath = []; hStart = []; hGoal = [];
if showEEPath
    hPath = plot3(ax, P(1,:), P(2,:), P(3,:), '-', 'Color', [0.10 0.10 0.10], 'LineWidth', 1.8);
    hStart = plot3(ax, P(1,1),   P(2,1),   P(3,1),   'o', 'MarkerSize', 8, ...
                   'MarkerFaceColor', [0.20 0.70 0.25], 'MarkerEdgeColor', 'k');
    hGoal  = plot3(ax, P(1,end), P(2,end), P(3,end), 's', 'MarkerSize', 8, ...
                   'MarkerFaceColor', [0.90 0.15 0.15], 'MarkerEdgeColor', 'k');
end
if showEEAxis
    quiver3(ax, P(1, idx), P(2, idx), P(3, idx), ...
            A(1, idx), A(2, idx), A(3, idx), axisLen, ...
            'Color', [0.93 0.55 0.05], 'LineWidth', 1.6, 'MaxHeadSize', 0.8, ...
            'AutoScale', 'off');
end

%% Look (same for every task)
axis(ax, scene.axisLimits);
view(ax, scene.viewAngle(1), scene.viewAngle(2));
grid(ax, 'on');
title(ax, figTitle, 'Interpreter', 'none');
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]'); zlabel(ax, 'z [m]');

% Small legend (only for entries that exist)
h = [hPath, hStart, hGoal, hObs];
names = {};
if ~isempty(hPath),  names{end+1} = 'EE path'; end
if ~isempty(hStart), names{end+1} = 'start';   end
if ~isempty(hGoal),  names{end+1} = 'goal';    end
if ~isempty(hObs),   names{end+1} = 'obstacle'; end
if ~isempty(h)
    legend(ax, h, names, 'Location', 'northeast');
end

%% Save
if ~isempty(saveFile)
    exportgraphics(fig, saveFile, 'Resolution', 200);
end
end


function v = getOpt(opts, name, default)
% GETOPT  opts.(name) if present and non-empty, otherwise the default.
if isfield(opts, name) && ~isempty(opts.(name))
    v = opts.(name);
else
    v = default;
end
end
