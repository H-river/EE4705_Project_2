function fig = plotCostHistory(results, labels, saveFile)
% PLOTCOSTHISTORY  Cost vs. iteration for one or several runs (for comparisons such as
% getTransform vs PoE, scenario 1/2/3, with vs without EE constraint).
%
% INPUT  results : 1 x K cell of result structs from runPipeline / runSTOMP
%                  (uses .Q_time, .RAR_time, optionally .iterTime)
%        labels  : 1 x K cell of legend strings
%        saveFile: (optional) '' = do not save
%
% Owner: T5

if nargin < 3, saveFile = ''; end
fig = figure('Name', 'cost_history');

%% TODO[T5-3b] (Owner: T5) Implement.
% Suggested layout (tiledlayout(1,2)):
%   left : total cost Q(theta) vs iteration (semilogy if ranges differ a lot)
%   right: control/smoothness cost vs iteration
% One line per run, legend(labels), consistent colours with plotConfigOverlay.
% if ~isempty(saveFile), exportgraphics(fig, saveFile); end
end
