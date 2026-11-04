function [fig, stats] = plotEEAxisDeviation(results, model, cfg, labels, saveFile)
% PLOTEEAXISDEVIATION  Angle between the constrained EE axis and cfg.worldAxis along
% each trajectory -- the main before/after figure of Task 5.
%
% INPUT  results : 1 x K cell of result structs from runPipeline / runSTOMP
%                  (a single struct is also accepted); uses .theta (nJoints x N)
%         model  : from buildRobotModel
%         cfg    : uses cfg.eeAxis (default 2) and cfg.worldAxis (default [0;0;1])
%         labels : (optional) 1 x K cell of legend strings
%         saveFile: (optional) '' = do not save, e.g. fullfile(cfg.outDir,'task5_axis_dev.png')
% OUTPUT fig     : figure handle
%         stats  : 1 x K struct array with fields label, maxDeg, meanDeg, angDeg
%                  (angDeg = 1 x N angles) -- for the report table
%
% Call (as in main_task5.m):
%   plotEEAxisDeviation({resOff, resOn}, model, cfg, {'no constraint','with constraint'}, file)
%
% Owner: T5

if nargin < 4, labels = {}; end
if nargin < 5 || isempty(saveFile), saveFile = ''; end

%% Normalise inputs
if isstruct(results), results = {results}; end
K = numel(results);
if isempty(labels)
    labels = arrayfun(@(k) sprintf('run %d', k), 1:K, 'UniformOutput', false);
end
if ischar(labels) || isstring(labels), labels = cellstr(labels); end
assert(numel(labels) == K, 'plotEEAxisDeviation: %d results but %d labels.', K, numel(labels));

eeAxis = 2;
if isfield(cfg, 'eeAxis') && ~isempty(cfg.eeAxis), eeAxis = cfg.eeAxis; end
worldAxis = [0; 0; 1];
if isfield(cfg, 'worldAxis') && ~isempty(cfg.worldAxis), worldAxis = cfg.worldAxis(:); end

axisName = 'xyz';
axisName = axisName(eeAxis);
if isequal(worldAxis(:)' / norm(worldAxis), [0 0 1])
    worldName = 'z';
else
    worldName = sprintf('[%g %g %g]', worldAxis / norm(worldAxis));
end

%% Colours: no constraint = orange, with constraint = blue, then extra runs
palette = [0.85 0.33 0.10;
           0.00 0.45 0.74;
           0.47 0.67 0.19;
           0.49 0.18 0.56;
           0.93 0.69 0.13;
           0.30 0.75 0.93];

%% Compute angles per waypoint
stats = struct('label', labels(:)', 'maxDeg', 0, 'meanDeg', 0, 'angDeg', []);
for k = 1:K
    ang = eeAxisAngles(model, results{k}.theta, eeAxis, worldAxis);
    stats(k).angDeg  = ang;
    stats(k).maxDeg  = max(ang);
    stats(k).meanDeg = mean(ang);
end

%% Plot
fig = figure('Name', 'ee_axis_deviation');
ax = axes(fig);
hold(ax, 'on');
grid(ax, 'on');
box(ax, 'on');

h = gobjects(1, K);
legendStr = cell(1, K);
degSign = char(176);
for k = 1:K
    col = palette(mod(k - 1, size(palette, 1)) + 1, :);
    ang = stats(k).angDeg;
    h(k) = plot(ax, 1:numel(ang), ang, '-o', 'LineWidth', 1.6, 'MarkerSize', 4, ...
                'Color', col, 'MarkerFaceColor', col);
    legendStr{k} = sprintf('%s (max %.1f%s, mean %.1f%s)', labels{k}, ...
                           stats(k).maxDeg, degSign, stats(k).meanDeg, degSign);
end

N = max(cellfun(@(r) size(r.theta, 2), results));
xlim(ax, [1 N]);
ylim(ax, [0, max(1, 1.1 * max([stats.maxDeg]))]);
xlabel(ax, 'Waypoint index');
ylabel(ax, sprintf('Angle between EE %s-axis and world %s-axis [deg]', axisName, worldName));
title(ax, sprintf('End-effector %s-axis deviation from world %s (0 = upright)', axisName, worldName));
lg = legend(ax, h, legendStr, 'Location', 'best');
lg.Interpreter = 'none';

%% Report numbers
fprintf('\n[plotEEAxisDeviation] EE %s-axis vs world %s\n', axisName, worldName);
for k = 1:K
    fprintf('  %-28s max %6.2f deg | mean %6.2f deg | start %5.2f | goal %5.2f\n', ...
        labels{k}, stats(k).maxDeg, stats(k).meanDeg, stats(k).angDeg(1), stats(k).angDeg(end));
end

%% Save
if ~isempty(saveFile)
    exportgraphics(fig, saveFile, 'Resolution', 200);
end
end
