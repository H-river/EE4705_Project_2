%% Task 1 -- KINOVA Gen3 + STOMP, original collision-avoidance setting
% Replaces KINOVA_STOMP_Path_Planning.m. Run this file from anywhere (F5).
% Owner: T12
%
% Done when: the planned path is collision-free (isTrajectoryInCollision = 0) and the
% animation plays, with getTransform FK.
%
% TODO tags for this task live in common/:
%   T1-1a..e runSTOMP.m | T1-2 updateJointsWorldPosition.m | T1-3 stompDTheta.m
%   T1-4 stompSamples.m | T1-5 stompObstacleCost.m | T1-6a/b bug fixes

clear; clc; close all;
thisDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisDir, '..', 'common')));

cfg = defaultConfig();
cfg.fkMode = 'getTransform';
cfg.outDir = fullfile(thisDir, 'results');

model = buildRobotModel('kinovaGen3', 'EndEffector_Link', cfg.fkMode);
scene = scene_kinova_default();

result = runPipeline(model, scene, cfg);

%% TODO[T1-7] (Owner: T12) Report figures for Task 1 (once T5-3 plotting is ready):
% plotConfigOverlay(model, result.theta, result.world, scene, struct('saveFile', fullfile(cfg.outDir,'task1_overlay.png')));
% plotCostHistory({result}, {'Kinova, getTransform'}, fullfile(cfg.outDir,'task1_cost.png'));
