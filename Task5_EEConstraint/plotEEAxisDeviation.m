function fig = plotEEAxisDeviation(results, model, cfg, labels, saveFile)
% PLOTEEAXISDEVIATION  Angle between the constrained EE axis and cfg.worldAxis along
% each trajectory -- the main before/after figure of Task 5.
% Owner: T5
if nargin < 5, saveFile = ''; end
fig = figure('Name', 'ee_axis_deviation');

%% TODO[T5-2d] (Owner: T5) For each result r and waypoint t:
%   [~, T] = updateJointsWorldPosition(model, r.theta(:, t));
%   a = T{end}(1:3, cfg.eeAxis);  ang(t) = acosd(max(-1, min(1, a' * cfg.worldAxis)));
% plot ang vs waypoint index, one line per run, legend(labels); report max / mean angle.
% if ~isempty(saveFile), exportgraphics(fig, saveFile); end
end
