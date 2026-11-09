%% compare_patience_all.m -- convergence patience P = 1 vs P = 3 on every project scene
% Same code and defaults as the tasks; only cfg.convergencePatience changes. Calls runSTOMP
% directly: no task script, no defaultConfig and no saved result is modified.
% PoE FK everywhere (identical to getTransform to ~1e-15, ~13x faster).

clear; clc; close all;
root = fileparts(fileparts(mfilename('fullpath')));          % this file lives in Bonus_STOMPExtensions/
addpath(genpath(fullfile(root, 'common')));

seeds    = 0:4;      % 5 runs per scene and P
patience = [1 3];
wConT5   = 10;       % Task 5 weight: replace with the w_con main_task5 chose (task5_weight_sweep.csv)

rc  = robotConfig();
kin = buildRobotModel('kinovaGen3', 'EndEffector_Link', 'PoE');
pan = buildRobotModel(rc.robotName, rc.eeName, 'PoE');
r5  = rc.sphereRadius;  m5 = rc.safetyMargin;
%     label          model  scene                  radius  margin  maxIter  w_con
C = { 'T1 Kinova',   kin,   @scene_kinova_default, 0.05,   0.05,   50,      0
    'T2/T3 Panda', pan,   @scene_newrobot_base,  r5,     m5,     50,      0
    'T4 S1',       pan,   @scene_t4_1,           r5,     m5,     200,     0
    'T4 S2',       pan,   @scene_t4_2,           r5,     m5,     200,     0
    'T4 S3',       pan,   @scene_t4_3,           r5,     m5,     200,     0
    'T5 no con',   pan,   @scene_t5,             r5,     m5,     200,     0
    'T5 con',      pan,   @scene_t5,             r5,     m5,     200,     wConT5 };

raw = table();
for s = 1:size(C, 1)
    model = C{s, 2};  scene = C{s, 3}();
    [q0, qT]    = solveInitFinal(model, scene);
    [vw, world] = buildVoxelWorld(scene);
    for P = patience
        cfg = defaultConfig();
        cfg.fkMode = 'PoE';  cfg.verbose = false;
        cfg.sphereRadius = C{s, 4};  cfg.safetyMargin = C{s, 5};
        cfg.maxIter = C{s, 6};
        cfg.useEEConstraint = C{s, 7} > 0;  cfg.w_con = C{s, 7};
        cfg.convergencePatience = P;
        cObs = cfg;  cObs.useEEConstraint = false;  cObs.w_con = 0;   % obstacle cost only
        [Rm, ~, ~] = stompSmoothingMatrices(cfg.nDiscretize, cfg.sigmaScale);
        for k = seeds
            rng(k);
            evalc('r = runSTOMP(model, q0, qT, vw, cfg);');          % silence per-run prints
            ok = ~any(checkTrajCollision(model.robot, r.theta, world)) && ...
                ~any(checkTrajCollisionDense(model.robot, r.theta, world, cfg.denseCheckSubsteps));
            Sobs = stompTrajCost(model, r.theta, Rm, vw, cObs);
            raw = [raw; table(string(C{s, 1}), P, k, ok, r.nIter, r.nIter >= cfg.maxIter, ...
                r.totalTime, sum(Sobs), 100 * meshClearance(model.robot, r.theta, world), ...
                'VariableNames', {'scene', 'P', 'seed', 'collisionFree', 'nIter', 'hitMaxIter', ...
                'time_s', 'obstacleCost', 'minClearance_cm'})]; %#ok<AGROW>
        end
        last = raw(end - numel(seeds) + 1:end, :);
        fprintf('%-12s P=%d | collision-free %d/%d | median iters %5.1f | median obstacle cost %7.2f | median clearance %5.2f cm\n', ...
            C{s, 1}, P, nnz(last.collisionFree), numel(seeds), median(last.nIter), ...
            median(last.obstacleCost), median(last.minClearance_cm));
    end
end

[g, sc, pp] = findgroups(raw.scene, raw.P);
summary = table(sc, pp, splitapply(@mean, raw.collisionFree, g), splitapply(@median, raw.nIter, g), ...
    splitapply(@sum, raw.hitMaxIter, g), splitapply(@median, raw.obstacleCost, g), ...
    splitapply(@median, raw.minClearance_cm, g), splitapply(@median, raw.time_s, g), ...
    'VariableNames', {'scene', 'P', 'successRate', 'medIters', 'runsAtMaxIter', ...
    'medObstacleCost', 'medClearance_cm', 'medTime_s'});
disp(summary)
outDir = fullfile(root, 'Bonus_STOMPExtensions', 'results');  if ~exist(outDir, 'dir'), mkdir(outDir); end
writetable(raw,     fullfile(outDir, 'patience_runs.csv'));
writetable(summary, fullfile(outDir, 'patience_summary.csv'));

function d = meshClearance(robot, theta, world)
% Smallest mesh distance robot <-> obstacles over all waypoints [m] (0 if any waypoint collides)
d = inf;
for t = 1:size(theta, 2)
    [hit, sep] = checkCollision(robot, theta(:, t), world, 'IgnoreSelfCollision', 'on', 'Exhaustive', 'on');
    if any(hit), d = 0; return; end
    d = min(d, min(sep(:), [], 'omitnan'));
end
end