function [tbl, raw] = stompAblation(outDir, seeds)
% STOMPABLATION  Effect of each STOMP extension (runSTOMP) on every project scene.
%
% Variants (cumulative, all with PoE FK):
%   no extensions : no joint limits, plain linspace init. Convergence patience stays at 3;
%                   P = 1 (the original stopping rule) is compared in compare_patience_all.
%   +limits       : samples and updates projected into the joint limits, shortest-angle init
%   +reuse        : + the 5 cheapest samples of the previous iteration are reused
%   +decay        : + exploration noise x 0.98^(iter-1) (floor 0.1)   [without reuse]
%   +reuse+decay  : both
% Scenes: Task 1 (Kinova), Task 2, Task 4 S1-S3, Task 5 without / with the constraint.
% Every (scene, variant) pair runs the same seeds; success = mesh-based collision check
% at the waypoints and between them (checkTrajCollisionDense).
%
% Outputs: bonus_ablation.csv (one row per scene x variant), bonus_ablation_runs.csv
% (one row per run), bonus_ablation.png.

if nargin < 1 || isempty(outDir), outDir = fullfile(fileparts(mfilename('fullpath')), 'results'); end
if nargin < 2 || isempty(seeds), seeds = 1:5; end
if ~exist(outDir, 'dir'), mkdir(outDir); end

V = {
 'no extensions', struct('enforceJointLimits', false, 'shortestAngleInit', false, 'nReuse', 0, 'noiseDecay', 1)
 '+limits',      struct('enforceJointLimits', true,  'shortestAngleInit', true,  'nReuse', 0, 'noiseDecay', 1)
 '+reuse',       struct('enforceJointLimits', true,  'shortestAngleInit', true,  'nReuse', 5, 'noiseDecay', 1)
 '+decay',       struct('enforceJointLimits', true,  'shortestAngleInit', true,  'nReuse', 0, 'noiseDecay', 0.98)
 '+reuse+decay', struct('enforceJointLimits', true,  'shortestAngleInit', true,  'nReuse', 5, 'noiseDecay', 0.98)
};
rc = robotConfig();
S = {   % label, robot, scene, maxIter, w_con (as in the task scripts)
 'T1 Kinova',  'kinova', @scene_kinova_default, 50,  0
 'T2 Panda',   'panda',  @scene_newrobot_base,  50,  0
 'T4 S1',      'panda',  @scene_t4_1,           200, 0
 'T4 S2',      'panda',  @scene_t4_2,           200, 0
 'T4 S3',      'panda',  @scene_t4_3,           200, 0
 'T5 no con',  'panda',  @scene_t5,             200, 0
 'T5 con',     'panda',  @scene_t5,             200, 10
};
models.kinova = buildRobotModel('kinovaGen3', 'EndEffector_Link', 'PoE');
models.panda  = buildRobotModel(rc.robotName, rc.eeName, 'PoE');

base = defaultConfig();
base.fkMode = 'PoE';  base.displayAnimation = false;  base.verbose = false;

raw = table();
for s = 1:size(S, 1)
    model = models.(S{s, 2});
    sc = S{s, 3}();
    [q0, qT] = solveInitFinal(model, sc);
    [vw, world] = buildVoxelWorld(sc);
    lo = model.jointLimits(:, 1);  hi = model.jointLimits(:, 2);
    for v = 1:size(V, 1)
        c = base;
        f = fieldnames(V{v, 2});
        for i = 1:numel(f), c.(f{i}) = V{v, 2}.(f{i}); end
        if strcmp(S{s, 2}, 'panda'), c.sphereRadius = rc.sphereRadius;  c.safetyMargin = rc.safetyMargin; end
        c.maxIter = S{s, 4};
        c.useEEConstraint = S{s, 5} > 0;  c.w_con = S{s, 5};
        for k = 1:numel(seeds)
            rng(seeds(k));
            evalc('r = runSTOMP(model, q0, qT, vw, c);');          % silence per-iteration prints
            viol = max([r.theta - hi; lo - r.theta], [], 'all');
            raw = [raw; table(string(S{s, 1}), string(V{v, 1}), seeds(k), ...
                ~any(checkTrajCollision(model.robot, r.theta, world)) && ...
                ~any(checkTrajCollisionDense(model.robot, r.theta, world, c.denseCheckSubsteps)), ...
                max(viol, 0), ...
                r.nIter, r.totalTime, maxTilt(model, r.theta), ...
                'VariableNames', {'scene', 'variant', 'seed', 'collisionFree', 'limitViolation_rad', ...
                                  'nIter', 'time_s', 'maxTilt_deg'})]; %#ok<AGROW>
        end
        last = raw(end - numel(seeds) + 1:end, :);
        fprintf('%-10s %-13s | collision-free %d/%d | max limit violation %.3f rad | median iters %5.1f | median time %.2f s\n', ...
            S{s, 1}, V{v, 1}, nnz(last.collisionFree), numel(seeds), max(last.limitViolation_rad), ...
            median(last.nIter), median(last.time_s));
    end
end

%% Summary: one row per scene x variant
[g, sceneCol, varCol] = findgroups(raw.scene, raw.variant);
tbl = table(sceneCol, varCol, splitapply(@mean, raw.collisionFree, g), ...
    splitapply(@max, raw.limitViolation_rad, g), splitapply(@median, raw.nIter, g), ...
    splitapply(@median, raw.time_s, g), splitapply(@median, raw.maxTilt_deg, g), ...
    'VariableNames', {'scene', 'variant', 'successRate', 'maxLimitViolation_rad', ...
                      'medianIters', 'medianTime_s', 'medianMaxTilt_deg'});
order = @(c, list) cellfun(@(x) find(strcmp(list, x)), cellstr(c));
[~, idx] = sortrows([order(tbl.scene, S(:, 1)), order(tbl.variant, V(:, 1))]);
tbl = tbl(idx, :);
disp(tbl);
writetable(tbl, fullfile(outDir, 'bonus_ablation.csv'));
writetable(raw, fullfile(outDir, 'bonus_ablation_runs.csv'));

%% Figure (also re-drawable from the CSV: plotAblation(readtable('bonus_ablation.csv'), ...))
plotAblation(tbl, S(:, 1), V(:, 1), numel(seeds), fullfile(outDir, 'bonus_ablation.png'));
end


function a = maxTilt(model, theta)
% Largest angle [deg] between the EE y-axis and world z along the trajectory (Task 5 metric)
a = 0;
for t = 1:size(theta, 2)
    [~, T] = updateJointsWorldPosition(model, theta(:, t));
    y = T{end}(1:3, 2);
    a = max(a, atan2d(norm(cross(y, [0; 0; 1])), y(3)));
end
end
