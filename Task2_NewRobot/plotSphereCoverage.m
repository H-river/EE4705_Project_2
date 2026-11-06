function fig = plotSphereCoverage(model, Q, radius, saveFile)
% PLOTSPHERECOVERAGE  Robot meshes with the STOMP collision spheres drawn on top, for
% one or more configurations (one tile each). Checks visually that the sphere model
% (stompRobotSphere: spheres along the straight segments between consecutive frames)
% covers every link of the chosen robot.
%
% INPUT  Q        : nJoints x K configurations (e.g. start, middle and goal waypoint)
%        radius   : sphere radius (scalar or 1 x nFK, as cfg.sphereRadius)
%        saveFile : (optional) image file, '' = do not save

if nargin < 4, saveFile = ''; end
K = size(Q, 2);
fig = figure('Name', 'sphere_coverage', 'Position', [100 100 420 * K 460]);
tl = tiledlayout(fig, 1, K, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('%s: STOMP collision spheres over the robot meshes', model.name), 'Interpreter', 'none');
[sx, sy, sz] = sphere(16);
for j = 1:K
    ax = nexttile(tl);
    show(model.robot, Q(:, j), 'Parent', ax, 'PreservePlot', false, 'Frames', 'off');
    set(findobj(ax, 'Type', 'patch'), 'FaceAlpha', 0.35);         % see-through meshes
    hold(ax, 'on');
    X =updateJointsWorldPosition(model, Q(:, j));
    [c, r] = stompRobotSphere(X, radius);
    for i = 1:size(c, 1)
        surf(ax, c(i,1) + r(i)*sx, c(i,2) + r(i)*sy, c(i,3) + r(i)*sz, ...
            'FaceColor', [0.00 0.45 0.74], 'FaceAlpha', 0.25, 'EdgeColor', 'none');
    end
    plot3(ax, X(:,1), X(:,2), X(:,3), 'k.-', 'MarkerSize', 12);   % frame origins / segments
    axis(ax, 'equal');  view(ax, 135, 20);  grid(ax, 'on');
    title(ax, sprintf('configuration %d (%d spheres)', j, size(c, 1)));
end
if ~isempty(saveFile)
    exportgraphics(fig, saveFile, 'Resolution', 200);
end
end
