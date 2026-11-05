%% Task 3 -- Replace getTransform with Product-of-Exponentials FK
% Owner: Jiamo (T34)
%
%   1. common/kinematics/computeScrewAxes.m + fkPoE.m   (TODO[T3-1], TODO[T3-2])
%   2. screwAxisTable.m : S_i table + matrix-log cross-check     (TODO[T3-1b/c])
%   3. compareFK.m      : accuracy + speed vs getTransform       (TODO[T3-3])
%   4. this script      : full STOMP run with PoE vs getTransform on the Task-2 robot,
%                         same seed -> same trajectory, per-iteration timing (TODO[T3-4])
%
% Outputs (Task3_PoE/results/): screw-axis CSVs, fk_compare.csv, plan_compare.csv,
% task3_cost.png, task3_overlay.png, and the usual runPipeline .mat files.

clear; clc; close all;
thisDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisDir, '..', 'common')));
addpath(thisDir);

useKinova = false;   % true = prototype on Kinova; false = Task-2 robot (robotConfig)
rc = robotConfig();
if useKinova || ~rc.isDecided
    robotName = 'kinovaGen3';  eeName = 'EndEffector_Link';  scene = scene_kinova_default();
    sphereRadius = 0.05;
else
    robotName = rc.robotName;  eeName = rc.eeName;  scene = scene_newrobot_base();
    sphereRadius = rc.sphereRadius;
end
outDir = fullfile(thisDir, 'results');
if ~exist(outDir, 'dir'), mkdir(outDir); end

%% 1) Screw axes (report table) for both robots
rng(1);
screwAxisTable(buildRobotModel(robotName, eeName, 'PoE'), ...
               fullfile(outDir, ['screw_axes_' robotName '.csv']));
if ~strcmp(robotName, 'kinovaGen3')
    screwAxisTable(buildRobotModel('kinovaGen3', 'EndEffector_Link', 'PoE'), ...
                   fullfile(outDir, 'screw_axes_kinovaGen3.csv'));
end

%% 2) FK accuracy / speed check (must pass before planning)
fkStats = compareFK(robotName, eeName);
if ~strcmp(robotName, 'kinovaGen3')
    fkStats(2) = compareFK('kinovaGen3', 'EndEffector_Link');
end
writetable(struct2table(fkStats), fullfile(outDir, 'fk_compare.csv'));
assert(all([fkStats.maxPosErr] < 1e-10) && all([fkStats.maxRotErr] < 1e-10), ...
    'Task 3: PoE does not match getTransform -- do not plan with it.');

%% 3) Plan with PoE and with getTransform on the same scene and the same seed
cfg = defaultConfig();
cfg.sphereRadius     = sphereRadius;
cfg.outDir           = outDir;
cfg.displayAnimation = false;

cfg.fkMode = 'PoE';
modelPoE = buildRobotModel(robotName, eeName, 'PoE');
rng(0); resPoE = runPipeline(modelPoE, scene, cfg);

cfg.fkMode = 'getTransform';
modelGT = buildRobotModel(robotName, eeName, 'getTransform');
rng(0); resGT = runPipeline(modelGT, scene, cfg);

%% TODO[T3-4] Planning-level comparison for the report
% Same seed -> same noise samples; PoE is exact to ~1e-15, so the trajectories must
% agree to round-off unless the tiny FK difference flips a voxel index (then they can
% drift apart a little -- report the number either way).
dTheta = max(abs(resPoE.theta - resGT.theta), [], 'all');
plan = table(["getTransform"; "PoE"], [resGT.nIter; resPoE.nIter], ...
    [mean(resGT.iterTime); mean(resPoE.iterTime)], [resGT.totalTime; resPoE.totalTime], ...
    [resGT.Q_time(end); resPoE.Q_time(end)], ...
    [resGT.isTrajectoryInCollision; resPoE.isTrajectoryInCollision], ...
    'VariableNames', {'fkMode', 'nIter', 'meanIterTime_s', 'totalTime_s', 'finalCost', 'inCollision'});
disp(plan);
fprintf('max |theta_PoE - theta_GT| = %.3e rad\n', dTheta);
fprintf('mean iter time: getTransform %.3f s | PoE %.3f s | speed-up x%.1f\n', ...
    mean(resGT.iterTime), mean(resPoE.iterTime), mean(resGT.iterTime) / mean(resPoE.iterTime));
writetable(plan, fullfile(outDir, 'plan_compare.csv'));

plotCostHistory({resGT, resPoE}, {'getTransform', 'PoE'}, fullfile(outDir, 'task3_cost.png'));
plotConfigOverlay(modelPoE, resPoE.theta, resPoE.world, scene, ...
    struct('title', sprintf('Task 3: %s, PoE FK', robotName), ...
           'saveFile', fullfile(outDir, 'task3_overlay.png')));
