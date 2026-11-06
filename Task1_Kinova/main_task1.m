%% Task 1 -- KINOVA Gen3 + STOMP, original collision-avoidance setting
% Replaces KINOVA_STOMP_Path_Planning.m. Run this file from anywhere (F5).
% Owner: Joshua Lim
%
% Done when: the planned path is collision-free (isTrajectoryInCollision = 0) and the
% animation plays, with getTransform FK.
%
% Completed sample-code functions (in common/): runSTOMP.m (was helperSTOMP.m),
%   updateJointsWorldPosition.m, stompDTheta.m, stompSamples.m, stompObstacleCost.m.
% Outputs (Task1_Kinova/results/): task1_overlay.png, task1_cost.png, runPipeline .mat.

clear; clc; close all;
thisDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisDir, '..', 'common')));

cfg = defaultConfig();
cfg.fkMode = 'getTransform';
cfg.outDir = fullfile(thisDir, 'results');

model = buildRobotModel('kinovaGen3', 'EndEffector_Link', cfg.fkMode);
scene = scene_kinova_default();

rng(0);
result = runPipeline(model, scene, cfg);

%% Report figures
plotConfigOverlay(model, result.theta, result.world, scene, struct( ...
    'title', 'Task 1: Kinova Gen3, getTransform', ...
    'saveFile', fullfile(cfg.outDir, 'task1_overlay.png')));
plotCostHistory({result}, {'Kinova, getTransform'}, fullfile(cfg.outDir, 'task1_cost.png'));
