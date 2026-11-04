%% Task 5 -- End-effector orientation constraint ("carry a cup of coffee")
% Owner: T5
%
% Cost:  common/stomp/stompConstraintCost.m      (TODO[T5-1])
% Scene: common/scenes/scene_t5.m                (TODO[T5-2a], upright start/goal)
% Shared plots: common/viz/plotConfigOverlay.m, plotCostHistory.m (TODO[T5-3a/b])
% Same scene, same seed, run twice: without and with the constraint.

clear; clc; close all;
thisDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisDir, '..', 'common')));
addpath(thisDir);

rc = robotConfig();
cfg = defaultConfig();
cfg.fkMode       = 'getTransform';            % switch to 'getTransform' until T3 is done
cfg.sphereRadius = rc.sphereRadius;
cfg.outDir       = fullfile(thisDir, 'results');
cfg.displayAnimation = false;
cfg.eeAxis    = 2;                   % EE y-axis
cfg.worldAxis = [0; 0; 1];           % kept upright

model = buildRobotModel(rc.robotName, rc.eeName, cfg.fkMode);
scene = scene_t5();

%% Without constraint
cfgOff = cfg;  cfgOff.useEEConstraint = false;
scene.name = 't5_noCon';
rng(0); resOff = runPipeline(model, scene, cfgOff);

%% With constraint -- weight sweep (TODO[T5-2b])
cfgOn = cfg;  cfgOn.useEEConstraint = true;  cfgOn.verbose = false;
wList  = [0 10 30 100 300 1000];
maxDev = zeros(size(wList));
for i = 1:numel(wList)
    c = cfgOn;  c.w_con = wList(i);
    scn = scene;  scn.name = sprintf('t5_w%g', wList(i));
    rng(0);  r = runPipeline(model, scn, c);  close all;
    ang = zeros(1, size(r.theta, 2));
    for t = 1:size(r.theta, 2)
        [~, T] = updateJointsWorldPosition(model, r.theta(:, t));
        ang(t) = acosd(max(-1, min(1, T{end}(1:3, cfg.eeAxis)' * cfg.worldAxis(:))));
    end
    maxDev(i) = max(ang);
    fprintf('w_con = %5g | max tilt %6.2f deg | collision %d | iters %d\n', ...
        wList(i), maxDev(i), r.isTrajectoryInCollision, r.nIter);
end

% Pick a value from the printed lines, then set it here and run the final case:
cfgOn.w_con = 100;  cfgOn.verbose = true;     % <- replace 100 with your choice
scene.name = 't5_withCon';
rng(0); resOn = runPipeline(model, scene, cfgOn);

%% T5-2c Before/after comparison (the key Task-5 result)
runs   = {resOff, resOn};
labels = {'no constraint', sprintf('with constraint (w_con = %g)', cfgOn.w_con)};
tags   = {'no_constraint', 'with_constraint'};

% (1) Tilt of the constrained EE axis at every waypoint  (plotEEAxisDeviation.m, T5-2d)
plotEEAxisDeviation(runs, model, cfg, labels, fullfile(cfg.outDir, 'task5_axis_deviation.png'));

% (2) Configuration overlays, one figure per run, same view (shared plot, T5-3a)
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
maxTilt = zeros(nRuns, 1);  meanTilt = zeros(nRuns, 1);  pathLen = zeros(nRuns, 1);
obsCost = zeros(nRuns, 1);  nIter = zeros(nRuns, 1);     totTime = zeros(nRuns, 1);
collides = false(nRuns, 1);
for k = 1:nRuns
    th = runs{k}.theta;
    ang = eeAxisAngles(model, th, cfg.eeAxis, cfg.worldAxis);
    maxTilt(k)  = max(ang);
    meanTilt(k) = mean(ang);
    pathLen(k)  = eePathLength(model, th);
    Sobs        = stompTrajCost(model, th, Rmat, voxelWorld, cfgObs);   % 1st output = Stheta
    obsCost(k)  = sum(Sobs);
    nIter(k)    = runs{k}.nIter;
    totTime(k)  = runs{k}.totalTime;
    collides(k) = runs{k}.isTrajectoryInCollision;
end
cmp = table(maxTilt, meanTilt, pathLen, obsCost, nIter, totTime, collides, ...
    'RowNames', tags, ...
    'VariableNames', {'maxTilt_deg', 'meanTilt_deg', 'EEpathLen_m', 'obstacleCost', ...
                      'iterations', 'time_s', 'inCollision'});
disp(cmp);
writetable(cmp, fullfile(cfg.outDir, 'task5_comparison.csv'), 'WriteRowNames', true);

% (4) Optional: cost curves of both runs (shared plot, T5-3b)
% plotCostHistory(runs, labels, fullfile(cfg.outDir, 'task5_cost.png'));


function L = eePathLength(model, theta)
% EEPATHLENGTH  Length [m] of the end-effector path through the waypoints.
N = size(theta, 2);
P = zeros(3, N);
for t = 1:N
    T = getTransform(model.robot, theta(:, t), model.eeName);
    P(:, t) = T(1:3, 4);
end
L = sum(vecnorm(diff(P, 1, 2), 2, 1));
end
