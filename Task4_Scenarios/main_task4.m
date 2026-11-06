%% Task 4 -- Custom collision-avoidance scenarios
% Owner: Jiamo (T34)
%
% Scenarios: common/scenes/scene_t4_1.m .. scene_t4_3.m
% Multi-obstacle voxelisation: common/world/buildVoxelWorld.m
% Uses the Task-2 robot + PoE FK (Tasks 3/4 build on each other).
%
% For every scenario:
%   1. one "showcase" run through runPipeline (rng 0): figures, .mat, optional video;
%   2. a robustness study: STOMP from the same start/goal with nSeeds
%      different seeds, success = mesh-based collision check finds no collision, at
%      the waypoints and at the sub-steps between them.
% Outputs (Task4_Scenarios/results/): task4_summary.csv, task4_seeds.csv,
% task4_<scene>_overlay.png, task4_cost.png, runPipeline .mat (+ .avi if makeVideos).

clear; clc; close all;
thisDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisDir, '..', 'common')));

makeVideos = false;   % true -> result video per scenario (for the slides)
nSeeds     = 10;      % robustness study

rc = robotConfig();
cfg = defaultConfig();
cfg.fkMode       = 'PoE';
cfg.sphereRadius = rc.sphereRadius;
cfg.safetyMargin = rc.safetyMargin;
cfg.outDir       = fullfile(thisDir, 'results');
cfg.displayAnimation  = false;
cfg.recordResultVideo = makeVideos;
cfg.maxIter      = 200;  % these scenes need a longer detour than Task 1/2 (50 iterations
                         % left 1-3 colliding waypoints in S1/S3; see the scene files)
if ~exist(cfg.outDir, 'dir'), mkdir(cfg.outDir); end

model = buildRobotModel(rc.robotName, rc.eeName, cfg.fkMode);

sceneFns = {@scene_t4_1, @scene_t4_2, @scene_t4_3};
nS = numel(sceneFns);
results = cell(1, nS);
labels  = cell(1, nS);
summary = table();
seedRows = table();

for s = 1:nS
    scene = sceneFns{s}();
    labels{s} = sprintf('S%d', s);

    %% 1) Showcase run
    rng(0);
    res = runPipeline(model, scene, cfg);
    results{s} = res;

    % Handout: "Do not initialize trivial paths, i.e., collision-free initialization."
    if ~res.initInCollision
        warning('%s: straight-line initialisation is collision-free -> scenario too easy.', scene.name);
    end

    [voxel_world] = buildVoxelWorld(scene);
    clearance = minClearance(model, res.theta, voxel_world, cfg.sphereRadius);

    plotConfigOverlay(model, res.theta, res.world, scene, struct( ...
        'title', sprintf('Task 4 %s: %s', labels{s}, scene.description), ...
        'saveFile', fullfile(cfg.outDir, sprintf('task4_%s_overlay.png', scene.name))));

    %% 2) Robustness over seeds (no figures; same start / goal / voxel world)
    [q0, qT] = solveInitFinal(model, scene);
    cfgR = cfg;  cfgR.verbose = false;
    ok = false(nSeeds, 1);  it = zeros(nSeeds, 1);  tt = zeros(nSeeds, 1);  nColl = zeros(nSeeds, 1);
    for k = 1:nSeeds
        rng(k);
        evalc('r = runSTOMP(model, q0, qT, voxel_world, cfgR);');   % silence per-run prints
        c = checkTrajCollision(model.robot, r.theta, res.world);
        cb = checkTrajCollisionDense(model.robot, r.theta, res.world, cfg.denseCheckSubsteps);
        ok(k) = ~any(c) && ~any(cb);  nColl(k) = nnz(c);  it(k) = r.nIter;  tt(k) = r.totalTime;
    end
    fprintf('[%s] robustness: %d/%d seeds collision-free | median iters %g | median time %.1f s\n', ...
        scene.name, nnz(ok), nSeeds, median(it), median(tt));
    seedRows = [seedRows; table(repmat(string(scene.name), nSeeds, 1), (1:nSeeds)', ok, nColl, it, tt, ...
        'VariableNames', {'scene', 'seed', 'collisionFree', 'collidingWaypoints', 'nIter', 'time_s'})]; %#ok<AGROW>

    %% Summary row
    types = strjoin(unique({scene.obstacles.type}), '+');
    initFlags = checkTrajCollision(model.robot, res.thetaInit, res.world);
    summary = [summary; table(string(labels{s}), string(scene.name), string(scene.description), ...
        numel(scene.obstacles), string(types), nnz(initFlags), res.isTrajectoryInCollision, ...
        res.nIter, res.totalTime, res.Q_time(end), clearance, nnz(ok) / nSeeds, median(it), median(tt), ...
        'VariableNames', {'id', 'scene', 'description', 'nObstacles', 'obstacleTypes', ...
        'initCollidingWaypoints', 'resultCollides', 'nIter', 'totalTime_s', 'finalCost', ...
        'minSphereClearance_m', 'successRate', 'medianIters', 'medianTime_s'})]; %#ok<AGROW>
end

%% Results table + figures for report / slides
disp(summary(:, setdiff(summary.Properties.VariableNames, {'description'}, 'stable')));
writetable(summary,  fullfile(cfg.outDir, 'task4_summary.csv'));
writetable(seedRows, fullfile(cfg.outDir, 'task4_seeds.csv'));
plotCostHistory(results, labels, fullfile(cfg.outDir, 'task4_cost.png'));


function d = minClearance(model, theta, voxel_world, radius)
% Smallest (signed distance - sphere radius) over all collision spheres and waypoints,
% using the same sphere / sEDT model STOMP optimises (> 0: no sphere touches an obstacle).
d = inf;
for i = 1:size(theta, 2)
    X = updateJointsWorldPosition(model, theta(:, i));
    [cen, rad] = stompRobotSphere(X, radius);
    idx = ceil((cen - voxel_world.Env_size(1, :)) ./ voxel_world.voxel_size);
    idx = min(max(idx, 1), voxel_world.world_size);
    sd  = voxel_world.sEDT(sub2ind(voxel_world.world_size, idx(:, 1), idx(:, 2), idx(:, 3)));
    d = min(d, min(sd - rad));
end
end
