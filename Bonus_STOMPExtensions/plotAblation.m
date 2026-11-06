function fig = plotAblation(tbl, sceneNames, variantNames, nSeeds, saveFile)
% PLOTABLATION  Bar charts of the STOMP-extension ablation (stompAblation output table):
% collision-free rate, median iterations and worst joint-limit violation, one group of
% bars per scene, one bar per variant.
if nargin < 5, saveFile = ''; end
sceneNames = cellstr(sceneNames);  variantNames = cellstr(variantNames);
nS = numel(sceneNames);  nV = numel(variantNames);
[sr, it, lv] = deal(nan(nS, nV));
for i = 1:nS
    for j = 1:nV
        r = strcmp(string(tbl.scene), sceneNames{i}) & strcmp(string(tbl.variant), variantNames{j});
        if any(r)
            sr(i, j) = 100 * tbl.successRate(r);
            it(i, j) = tbl.medianIters(r);
            lv(i, j) = rad2deg(tbl.maxLimitViolation_rad(r));
        end
    end
end
fig = figure('Name', 'bonus_ablation', 'Position', [100 100 1500 480]);
tl = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('STOMP extensions (%d seeds per bar)', nSeeds));
data  = {sr, it, lv};
ylabs = {'Collision-free runs [%]', 'Median iterations', 'Worst joint-limit violation [deg]'};
for k = 1:3
    ax = nexttile(tl);
    bar(ax, data{k});
    grid(ax, 'on');
    set(ax, 'XTick', 1:nS, 'XTickLabel', sceneNames, 'XTickLabelRotation', 30);
    ylabel(ax, ylabs{k});
    if k == 1, ylim(ax, [0 105]); end
end
lg = legend(ax, variantNames, 'Orientation', 'horizontal');
lg.Layout.Tile = 'south';
if ~isempty(saveFile)
    exportgraphics(fig, saveFile, 'Resolution', 200);
end
end
