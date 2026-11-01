function [fig, ax] = initVisualizer(model, q0, poseFinal, world, scene)
% INITVISUALIZER  Robot at q0, goal marker, obstacles (was helperInitialVisualizerKINOVA).
fig = figure('Position', [375 446 641 480], 'Name', scene.name);
ax = show(model.robot, q0, 'PreservePlot', false, 'Frames', 'off');
view(scene.viewAngle(1), scene.viewAngle(2));
hold on
axis(scene.axisLimits);
% goal position
plot3(poseFinal(1), poseFinal(2), poseFinal(3), 'r.', 'MarkerSize', 20);
% obstacles
for i = 1:numel(world)
    [~, pObj] = show(world{i}, 'Parent', ax);
    pObj.LineStyle = 'none';
end
end
