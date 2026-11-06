%% Task 2 -- STOMP on a different manipulator, scenario similar to Task 1
% Owner: Joshua Lim
%
% Robot: Franka Emika Panda, set in common/config/robotConfig.m; scene:
% common/scenes/scene_newrobot_base.m. Tasks 3-5 use the same robot via robotConfig.
% Outputs (Task2_NewRobot/results/): task2_spheres.png (collision-sphere coverage),
% task2_overlay.png, task2_cost.png, runPipeline .mat.

clear; clc; close all;
thisDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisDir, '..', 'common')));
addpath(thisDir);                                   % plotSphereCoverage

rc = robotConfig();
if ~rc.isDecided
    warning('Task 2 robot not chosen yet (robotConfig.isDecided = false) -- running Kinova.');
end

cfg = defaultConfig();
cfg.fkMode       = 'getTransform';
cfg.sphereRadius = rc.sphereRadius;
cfg.safetyMargin = rc.safetyMargin;
cfg.outDir       = fullfile(thisDir, 'results');

model = buildRobotModel(rc.robotName, rc.eeName, cfg.fkMode);
scene = scene_newrobot_base();

% Sanity check: only movable joints, in configuration order (fixed bodies and the frozen
% gripper fingers are excluded by buildRobotModel); the EE is the last FK frame.
fprintf('Movable joints (%d): %s\n', model.nJoints, strjoin(model.jointBodyNames, ', '));
fprintf('Joint types: %s | FK frames: %d (last = %s)\n', ...
    strjoin(unique(model.jointTypes), ', '), model.nFK, model.fkBodyNames{end});

% Collision-sphere radii: re-fit them to the meshes and compare with robotConfig
[radFit, radInfo] = fitSphereRadii(model);
disp(radInfo);
writetable(radInfo, fullfile(cfg.outDir, 'task2_sphere_radii.csv'));
fprintf('robotConfig radii: %s\nfitted (p99)     : %s\n', mat2str(rc.sphereRadius, 3), mat2str(radFit, 3));

rng(0);
result = runPipeline(model, scene, cfg);

%% Report figures
% Collision-sphere coverage at the start, middle and goal waypoint (are all links covered?)
N = size(result.theta, 2);
plotSphereCoverage(model, result.theta(:, [1 round(N/2) N]), cfg.sphereRadius, ...
    fullfile(cfg.outDir, 'task2_spheres.png'));
plotConfigOverlay(model, result.theta, result.world, scene, struct( ...
    'title', sprintf('Task 2: %s, getTransform', rc.robotName), ...
    'saveFile', fullfile(cfg.outDir, 'task2_overlay.png')));
plotCostHistory({result}, {sprintf('%s, getTransform', rc.robotName)}, ...
    fullfile(cfg.outDir, 'task2_cost.png'));
