%% Task 5 -- End-effector orientation constraint ("carry a cup of coffee")
% Owner: T5
%
% Cost:  common/stomp/stompConstraintCost.m
% Scene: common/scenes/scene_t5.m                (upright start/goal)
% Shared plots: common/viz/plotConfigOverlay.m, plotCostHistory.m
% Same scene, same seed, run twice: without and with the constraint.

clear; clc; close all;
thisDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisDir, '..', 'common')));
addpath(thisDir);

rc = robotConfig();
cfg = defaultConfig();
cfg.fkMode       = 'PoE';                     % Task-3 FK, ~13x faster sweep than getTransform
cfg.sphereRadius = rc.sphereRadius;
cfg.safetyMargin = rc.safetyMargin;
cfg.maxIter      = 200;                       % 50 stopped every constrained run early
                                              % (some still colliding); 200 as in Task 4
cfg.outDir       = fullfile(thisDir, 'results');
cfg.displayAnimation = false;
cfg.eeAxis    = 2;                   % EE y-axis
cfg.worldAxis = [0; 0; 1];           % kept upright

if ~exist(cfg.outDir, 'dir'), mkdir(cfg.outDir); end

model = buildRobotModel(rc.robotName, rc.eeName, cfg.fkMode);
scene = scene_t5();

%% Without constraint
cfgOff = cfg;  cfgOff.useEEConstraint = false;
scene.name = 't5_noCon';
rng(0); resOff = runPipeline(model, scene, cfgOff);

%% With constraint -- weight sweep
% STOMP is stochastic (mvnrnd noise), so ONE run per weight can mislead: a lucky seed can
% make a weak weight look good. Every weight is therefore run with several seeds and
% judged on the median / worst case. The sweep calls runSTOMP directly (not runPipeline)
% so no figures, animations or .mat files are produced per run; IK and the voxel world
% are computed once because they do not depend on w_con.
%
% Toolboxes used here (all already required by the project):
%   Robotics System Toolbox : getTransform / tform2rotm (EE tilt), checkCollision (via
%                             checkTrajCollision, mesh-based ground truth)
%   Statistics & ML Toolbox : boxplot (spread of max tilt over seeds)
% Not used: Image Processing (only needed for the sEDT, already in buildVoxelWorld)

wList      = [0 10 30 100 300 1000 3000];   % 0 = constraint switched on but weight 0 (sanity)
seedList   = 0:4;                           % seed 0 is the seed used in the final runs
tiltTolDeg = 5;                             % acceptable worst-case tilt of the EE y-axis [deg]

cfgOn = cfg;  cfgOn.useEEConstraint = true;  cfgOn.verbose = false;

[q0, qT]          = solveInitFinal(model, scene);
[voxelWorld, wld] = buildVoxelWorld(scene);

nW = numel(wList);  nS = numel(seedList);
maxTilt  = nan(nW, nS);   meanTilt = nan(nW, nS);
pathLenS = nan(nW, nS);   nIterS   = nan(nW, nS);
collideS = false(nW, nS);

fprintf('\n--- Weight sweep: %d weights x %d seeds = %d STOMP runs ---\n', nW, nS, nW * nS);
for i = 1:nW
    c = cfgOn;  c.w_con = wList(i);
    for s = 1:nS
        rng(seedList(s));
        res = runSTOMP(model, q0, qT, voxelWorld, c);
        ang = eeTilt(model, res.theta, cfg.eeAxis, cfg.worldAxis);
        maxTilt(i, s)  = max(ang);
        meanTilt(i, s) = mean(ang);
        pathLenS(i, s) = eePathLength(model, res.theta);
        nIterS(i, s)   = res.nIter;
        collideS(i, s) = any(checkTrajCollision(model.robot, res.theta, wld)) || ...
                         any(checkTrajCollisionDense(model.robot, res.theta, wld, cfg.denseCheckSubsteps));
    end
    fprintf('w_con = %5g | max tilt: median %6.2f, worst %6.2f deg | collision-free %d/%d | median iters %g\n', ...
        wList(i), median(maxTilt(i, :)), max(maxTilt(i, :)), nnz(~collideS(i, :)), nS, median(nIterS(i, :)));
end

% Summary table (one row per weight)
okRate = mean(~collideS, 2);
sweepTbl = table(wList(:), okRate, median(maxTilt, 2), max(maxTilt, [], 2), ...
    median(meanTilt, 2), median(pathLenS, 2), median(nIterS, 2), ...
    'VariableNames', {'w_con', 'collisionFreeRate', 'medMaxTilt_deg', 'worstMaxTilt_deg', ...
                      'medMeanTilt_deg', 'medEEpathLen_m', 'medIterations'});
disp(sweepTbl);
writetable(sweepTbl, fullfile(cfg.outDir, 'task5_weight_sweep.csv'));

% Automatic choice: the SMALLEST positive weight that is collision-free for every seed
% and keeps the median worst-case tilt below tiltTolDeg. Smallest = the constraint does
% not drown the obstacle term (see stompUpdateProb: costs are min-max normalised per time
% step, so only the ratio w_con*qc : w_obs*qo matters).
medTilt = median(maxTilt, 2);
ok = (okRate == 1) & (medTilt <= tiltTolDeg) & (wList(:) > 0);
if any(ok)
    iBest = find(ok, 1, 'first');
else
    warning(['No weight is collision-free on all seeds AND below %g deg tilt. ' ...
             'Falling back to the lowest-tilt weight among those with the best collision-free rate. ' ...
             'Consider a larger wList, more iterations, or a different tolerance.'], tiltTolDeg);
    pos  = wList(:) > 0;                                  % best rate AMONG positive weights
    cand = find(pos & (okRate == max(okRate(pos))));      % (max over all picked w = 0 -> empty)
    [~, j] = min(medTilt(cand));
    iBest = cand(j);
end
wBest = wList(iBest);
fprintf('Chosen w_con = %g\n', wBest);

% Sweep figure for the report: tilt spread over seeds, and collision-free rate
wLabels = arrayfun(@(w) num2str(w), wList, 'UniformOutput', false);
figSw = figure('Name', 't5_weight_sweep', 'Position', [100 100 1000 420]);
tl = tiledlayout(figSw, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Task 5 weight sweep (%d seeds per weight)', nS));

ax1 = nexttile(tl);                                  % boxplot draws into the current axes
boxplot(maxTilt', 'Labels', wLabels);                % Statistics & ML Toolbox; one box per weight
hold(ax1, 'on');
yline(ax1, tiltTolDeg, '--r', sprintf('tolerance %g deg', tiltTolDeg), 'LabelHorizontalAlignment', 'left');
xlabel(ax1, 'w_{con}');  ylabel(ax1, 'Max tilt of EE y-axis [deg]');
title(ax1, sprintf('Constraint accuracy (chosen w_{con} = %g)', wBest));
grid(ax1, 'on');

ax2 = nexttile(tl);
bar(ax2, 100 * okRate, 'FaceColor', [0.00 0.45 0.74]);
set(ax2, 'XTick', 1:nW, 'XTickLabel', wLabels);
ylim(ax2, [0 105]);
xlabel(ax2, 'w_{con}');  ylabel(ax2, 'Collision-free runs [%]');
title(ax2, 'Obstacle avoidance (mesh-based check)');
grid(ax2, 'on');
exportgraphics(figSw, fullfile(cfg.outDir, 'task5_weight_sweep.png'), 'Resolution', 200);

% Final with-constraint run, same seed as the no-constraint run. To override the
% automatic choice, replace wBest with a number read from the table above.
cfgOn.w_con = wBest;  cfgOn.verbose = true;
scene.name = 't5_withCon';
rng(0); resOn = runPipeline(model, scene, cfgOn);

%% Before/after comparison (the key Task-5 result)
runs   = {resOff, resOn};
labels = {'no constraint', sprintf('with constraint (w_con = %g)', cfgOn.w_con)};
tags   = {'no_constraint', 'with_constraint'};

% (1) Tilt of the constrained EE axis at every waypoint  (plotEEAxisDeviation.m)
[~, devStats] = plotEEAxisDeviation(runs, model, cfg, labels, ...
    fullfile(cfg.outDir, 'task5_axis_deviation.png'));   % devStats(k).maxDeg / .meanDeg / .angDeg

% (2) Configuration overlays, one figure per run, same view (shared plot)
plotConfigOverlay(model, resOff.theta, resOff.world, resOff.scene, ...
    struct('title', 'No constraint', 'saveFile', fullfile(cfg.outDir, 'task5_overlay_noCon.png')));
plotConfigOverlay(model, resOn.theta, resOn.world, resOn.scene, ...
    struct('title', 'With constraint', 'saveFile', fullfile(cfg.outDir, 'task5_overlay_withCon.png')));

% (3) What the constraint costs: iterations, time, EE path length, obstacle cost, collisions.
%     Obstacle cost is re-evaluated for BOTH runs with the same obstacle-only settings
%     (constraint term switched off), otherwise the "with" run would include w_con*qc.
cfgObs = cfg;  cfgObs.useEEConstraint = false;  cfgObs.w_con = 0;
[Rmat, ~, ~] = stompSmoothingMatrices(cfg.nDiscretize, cfg.sigmaScale);
voxelWorld = buildVoxelWorld(resOff.scene);        % both runs use the same obstacles

nRuns = numel(runs);
maxTilt2 = zeros(nRuns, 1);  meanTilt2 = zeros(nRuns, 1);  pathLen = zeros(nRuns, 1);
obsCost = zeros(nRuns, 1);   nIter = zeros(nRuns, 1);      totTime = zeros(nRuns, 1);
collides = false(nRuns, 1);
for k = 1:nRuns
    th = runs{k}.theta;
    maxTilt2(k)  = devStats(k).maxDeg;     % from plotEEAxisDeviation, no extra helper needed
    meanTilt2(k) = devStats(k).meanDeg;
    pathLen(k)   = eePathLength(model, th);
    Sobs         = stompTrajCost(model, th, Rmat, voxelWorld, cfgObs);   % 1st output = Stheta
    obsCost(k)   = sum(Sobs);
    nIter(k)     = runs{k}.nIter;
    totTime(k)   = runs{k}.totalTime;
    collides(k)  = runs{k}.isTrajectoryInCollision;
end
cmp = table(maxTilt2, meanTilt2, pathLen, obsCost, nIter, totTime, collides, ...
    'RowNames', tags, ...
    'VariableNames', {'maxTilt_deg', 'meanTilt_deg', 'EEpathLen_m', 'obstacleCost', ...
                      'iterations', 'time_s', 'inCollision'});
disp(cmp);
writetable(cmp, fullfile(cfg.outDir, 'task5_comparison.csv'), 'WriteRowNames', true);

% (4) Optional: cost curves of both runs (shared plot)
% plotCostHistory(runs, labels, fullfile(cfg.outDir, 'task5_cost.png'));


function ang = eeTilt(model, theta, eeAxis, worldAxis)
% EETILT  Angle [deg] between EE body axis eeAxis and worldAxis at every waypoint.
% Same formula as plotEEAxisDeviation (atan2d form), repeated here so the sweep does not
% need to open a figure per run.
w = worldAxis(:) / norm(worldAxis);
N = size(theta, 2);
ang = zeros(1, N);
for t = 1:N
    R = tform2rotm(getTransform(model.robot, theta(:, t), model.eeName));
    a = R(:, eeAxis);
    ang(t) = atan2d(norm(cross(a, w)), dot(a, w));
end
end


function L = eePathLength(model, theta)
% EEPATHLENGTH  Length [m] of the end-effector path through the waypoints.
N = size(theta, 2);
P = zeros(3, N);
for t = 1:N
    T = getTransform(model.robot, theta(:, t), model.eeName);   % Robotics System Toolbox
    P(:, t) = tform2trvec(T)';                                  % translation part of the pose
end
L = sum(vecnorm(diff(P, 1, 2), 2, 1));
end
