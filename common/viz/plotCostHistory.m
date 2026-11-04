function fig = plotCostHistory(results, labels, saveFile)
% PLOTCOSTHISTORY  Cost vs. iteration for one or several runs (for comparisons such as
% getTransform vs PoE, scenario 1/2/3, with vs without EE constraint).
%
% INPUT  results : 1 x K cell of result structs from runPipeline / runSTOMP
%                  (uses .Q_time, .RAR_time, optionally .iterTime);
%                  a single struct is also accepted
%        labels  : 1 x K cell of legend strings (a char / string is also accepted;
%                  default 'run 1', 'run 2', ...)
%        saveFile: (optional) '' = do not save
% OUTPUT fig     : figure handle
%
% Layout (tiledlayout(1,2)):
%   left : total trajectory cost Q(theta) vs iteration
%   right: control / smoothness cost (1/2 * theta' R theta) vs iteration
% One line per run, same colours as plotEEAxisDeviation (run 1 = orange,
% run 2 = blue, ...). Each axis switches to a log scale automatically when all values
% are positive and span more than a factor of 20.
%
% Note for Task 5: Q(theta) of a run WITH the EE constraint also contains the term
% w_con * qc, so it is not the same quantity as Q of the run without it. The control
% cost on the right is comparable across runs. For an obstacle-only comparison use the
% re-evaluated "obstacleCost" column in main_task5.
%
% Only base MATLAB is used (tiledlayout, plot, legend, exportgraphics), so this works
% on any result struct, including ones from the placeholder STOMP loop.
%
% Owner: T5

if nargin < 2, labels = {}; end
if nargin < 3 || isempty(saveFile), saveFile = ''; end

%% Normalise inputs
if isstruct(results), results = {results}; end
K = numel(results);
if isempty(labels)
    labels = arrayfun(@(k) sprintf('run %d', k), 1:K, 'UniformOutput', false);
end
if ischar(labels) || isstring(labels), labels = cellstr(labels); end
assert(numel(labels) == K, 'plotCostHistory: %d results but %d labels.', K, numel(labels));

%% Colours (same palette as plotEEAxisDeviation)
palette = [0.85 0.33 0.10;
           0.00 0.45 0.74;
           0.47 0.67 0.19;
           0.49 0.18 0.56;
           0.93 0.69 0.13;
           0.30 0.75 0.93];

%% Figure
fig = figure('Name', 'cost_history', 'Position', [100 100 1000 420]);
tl  = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'STOMP cost history');

axQ = nexttile(tl);
axR = nexttile(tl);

hQ = gobjects(1, K);
legendStr = cell(1, K);
allQ = [];
allR = [];
nMax = 1;

for k = 1:K
    col = palette(mod(k - 1, size(palette, 1)) + 1, :);
    res = results{k};

    Q = getSeries(res, 'Q_time');
    R = getSeries(res, 'RAR_time');
    if isempty(Q)
        warning('plotCostHistory: run "%s" has no Q_time -- nothing to plot for it.', labels{k});
    end

    hQ(k) = plotOne(axQ, Q, col);
    plotOne(axR, R, col);

    allQ = [allQ, Q]; %#ok<AGROW>
    allR = [allR, R]; %#ok<AGROW>
    nMax = max([nMax, numel(Q), numel(R)]);

    % legend text: label + iteration count (+ mean time per iteration if logged)
    legendStr{k} = sprintf('%s (%d it', labels{k}, numel(Q));
    if isfield(res, 'iterTime') && ~isempty(res.iterTime)
        legendStr{k} = sprintf('%s, %.3g s/it', legendStr{k}, mean(res.iterTime));
    end
    legendStr{k} = [legendStr{k} ')'];
end

%% Axes cosmetics
styleAxes(axQ, 'Total cost Q(\theta)',                  allQ, nMax);
styleAxes(axR, 'Control cost  ½ \Sigma \theta^T R \theta', allR, nMax);

lg = legend(axQ, hQ, legendStr, 'Location', 'best');
lg.Interpreter = 'none';

%% Save
if ~isempty(saveFile)
    exportgraphics(fig, saveFile, 'Resolution', 200);
end
end


function v = getSeries(res, name)
% GETSERIES  res.(name) as a row vector, [] if missing / empty.
if isfield(res, name) && ~isempty(res.(name))
    v = double(res.(name)(:))';
else
    v = [];
end
end


function h = plotOne(ax, y, col)
% PLOTONE  One run as a line (+ filled marker on the last iteration).
if isempty(y)
    h = plot(ax, NaN, NaN, '-', 'Color', col, 'LineWidth', 1.6);   % keeps legend entry
    hold(ax, 'on');
    return
end
h = plot(ax, 1:numel(y), y, '-', 'Color', col, 'LineWidth', 1.6);
hold(ax, 'on');
plot(ax, numel(y), y(end), 'o', 'MarkerSize', 6, 'MarkerFaceColor', col, ...
     'MarkerEdgeColor', col, 'HandleVisibility', 'off');
end


function styleAxes(ax, yLab, vals, nMax)
% STYLEAXES  Labels, grid, x-limits and automatic log scale.
grid(ax, 'on');
box(ax, 'on');
xlabel(ax, 'Iteration');
ylabel(ax, yLab);
xlim(ax, [1 max(nMax, 2)]);
vals = vals(isfinite(vals));
if ~isempty(vals) && all(vals > 0) && max(vals) / min(vals) > 20
    set(ax, 'YScale', 'log');          % ranges differ a lot -> log axis
end
end
