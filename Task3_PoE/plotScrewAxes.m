function fig = plotScrewAxes(model, saveFile)
% PLOTSCREWAXES  Space-frame screw axes S_i = [w_i; v_i] drawn on the robot at the zero
% configuration (the reference configuration of computeScrewAxes / fkPoE).
%
% For each joint: the joint origin q_i (a point on the axis, from getTransform), the
% axis direction w_i as an arrow, and the axis as a dashed line. The label gives w_i and
% q_i; v_i = -w_i x q_i (the "2D car" construction of the lecture, in 3D). The point
% q_i is re-derived from S_i as a check: w_i x v_i is the point of the axis closest to
% the base origin, and it must lie on the line through q_i along w_i.
%
% Note: the Panda zero configuration violates the limits of joints 4 and 6 -- it is only
% a reference pose for the PoE formula, the planner never goes there.

if nargin < 2, saveFile = ''; end
z = zeros(model.nJoints, 1);
S = model.S;

fig = figure('Name', 'screw_axes', 'Position', [100 100 900 750]);
ax = axes(fig);
show(model.robot, z, 'Parent', ax, 'PreservePlot', false, 'Frames', 'off');
set(findobj(ax, 'Type', 'patch'), 'FaceAlpha', 0.25);
hold(ax, 'on');
col = lines(model.nJoints);
L = 0.15;                                             % arrow length [m]
for i = 1:model.nJoints
    w = S(1:3, i);  v = S(4:6, i);
    Tb = getTransform(model.robot, z, model.jointBodyNames{i});
    Tj = Tb / getBody(model.robot, model.jointBodyNames{i}).Joint.ChildToJointTransform;
    q = Tj(1:3, 4);
    % check: closest axis point to the origin (w x v) lies on the line q + t w
    p0 = cross(w, v);
    assert(norm(cross(p0 - q, w)) < 1e-9, 'plotScrewAxes: S_%d inconsistent with the joint origin.', i);
    line = q + [-1 1] * L .* w;
    plot3(ax, line(1, :), line(2, :), line(3, :), '--', 'Color', col(i, :), 'LineWidth', 1);
    quiver3(ax, q(1), q(2), q(3), L * w(1), L * w(2), L * w(3), 0, 'Color', col(i, :), ...
        'LineWidth', 2.5, 'MaxHeadSize', 0.6);
    plot3(ax, q(1), q(2), q(3), 'o', 'Color', col(i, :), 'MarkerFaceColor', col(i, :), 'MarkerSize', 5);
    off = 0.03 * (mod(i, 2) * 2 - 1);                  % stagger labels of coincident joints
    text(ax, q(1) + L * w(1) + off, q(2) + L * w(2), q(3) + L * w(3) + off, ...
        sprintf('S_%d: \\omega=[%s], q=[%s]', i, fmt(w), fmt(q)), ...
        'Color', col(i, :) * 0.8, 'FontSize', 8, 'FontWeight', 'bold', ...
        'BackgroundColor', 'w', 'Margin', 0.5);
end
% show() fixes the camera to the robot; widen the box so that every arrow and label fits
Q = cell2mat(arrayfun(@(i) tform2trvec(getTransform(model.robot, z, model.jointBodyNames{i}))', ...
    1:model.nJoints, 'UniformOutput', false));
pad = 0.35;
set(ax, 'CameraViewAngleMode', 'auto', 'DataAspectRatio', [1 1 1], ...
    'XLim', [min(Q(1, :)) - pad, max(Q(1, :)) + pad], 'YLim', [min(Q(2, :)) - pad, max(Q(2, :)) + pad], ...
    'ZLim', [0, max(Q(3, :)) + pad]);
view(ax, 135, 20);  grid(ax, 'on');
title(ax, {sprintf('%s: space-frame screw axes at the zero configuration', model.name), ...
    'S_i = [\omega_i ; -\omega_i \times q_i]   (arrow = \omega_i at joint origin q_i)'}, 'Interpreter', 'tex');
if ~isempty(saveFile)
    exportgraphics(fig, saveFile, 'Resolution', 200);
end
end


function s = fmt(x)
x(abs(x) < 5e-4) = 0;
s = strjoin(arrayfun(@(a) sprintf('%.3g', a), x(:)', 'UniformOutput', false), ' ');
end
