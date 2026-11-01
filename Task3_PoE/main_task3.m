%% Task 3 -- Replace getTransform with Product-of-Exponentials FK
% Owner: Jiamo (T34)
%
% Order of work:
%   1. TODO[T3-1] computeScrewAxes  and  TODO[T3-2] fkPoE   (common/kinematics/)
%      -> prototype on Kinova first (useKinova = true), robot is already known.
%   2. compareFK.m : accuracy + speed vs getTransform          (TODO[T3-3])
%   3. this script: full STOMP run with PoE on the Task-2 robot and timing vs Task 2.

clear; clc; close all;
thisDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisDir, '..', 'common')));
addpath(thisDir);

useKinova = true;   % prototype on Kinova; set false once Task 2 robot is chosen
if useKinova
    robotName = 'kinovaGen3';  eeName = 'EndEffector_Link';  scene = scene_kinova_default();
    sphereRadius = 0.05;
else
    rc = robotConfig();
    robotName = rc.robotName;  eeName = rc.eeName;  scene = scene_newrobot_base();
    sphereRadius = rc.sphereRadius;
end

%% 1) FK accuracy / speed check (must pass before planning)
compareFK(robotName, eeName);

%% 2) Plan with PoE and with getTransform on the same scene
cfg = defaultConfig();
cfg.sphereRadius     = sphereRadius;
cfg.outDir           = fullfile(thisDir, 'results');
cfg.displayAnimation = false;

cfg.fkMode = 'PoE';
modelPoE = buildRobotModel(robotName, eeName, 'PoE');
rng(0); resPoE = runPipeline(modelPoE, scene, cfg);

cfg.fkMode = 'getTransform';
modelGT = buildRobotModel(robotName, eeName, 'getTransform');
rng(0); resGT = runPipeline(modelGT, scene, cfg);

%% TODO[T3-4] (Owner: Jiamo) Planning-level comparison for the report:
% - same rng seed -> trajectories should match to ~1e-10 if PoE is exact; report
%   max(abs(resPoE.theta - resGT.theta), [], 'all');
% - mean per-iteration time: mean(resPoE.iterTime) vs mean(resGT.iterTime), speed-up;
% - plotCostHistory({resGT, resPoE}, {'getTransform', 'PoE'}, ...).
fprintf('mean iter time: getTransform %.3f s | PoE %.3f s\n', mean(resGT.iterTime), mean(resPoE.iterTime));
