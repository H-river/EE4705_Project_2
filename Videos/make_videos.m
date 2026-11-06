%% Presentation videos (MPEG-4) from the saved results of every task
% Run the task scripts first (each saves its runPipeline .mat in <Task>/results/), then
% this script. Output: Videos/results/*.mp4 (~3 min in total).
%
%   task1_kinova.mp4             Task 1 trajectory
%   task1_optimisation.mp4       Task 1: STOMP iterations pushing the path off the box
%   task2_panda.mp4              Task 2 trajectory
%   task4_S1/S2/S3.mp4           Task 4 scenarios
%   task4_S3_optimisation.mp4    Task 4 S3: STOMP iterations
%   task5_compare.mp4            Task 5 without | with the EE constraint, EE y-axis in orange
%   bonus_wrap_compare.mp4       naive | shortest-angle initialisation (Kinova base across +-pi)
%
% Owner: Jiamo

clear; clc; close all;
thisDir = fileparts(mfilename('fullpath'));
root = fullfile(thisDir, '..');
addpath(genpath(fullfile(root, 'common')));
outDir = fullfile(thisDir, 'results');
if ~exist(outDir, 'dir'), mkdir(outDir); end

rc = robotConfig();
kinova = buildRobotModel('kinovaGen3', 'EndEffector_Link', 'PoE');
panda  = buildRobotModel(rc.robotName, rc.eeName, 'PoE');
ld = @(task, file) getfield(load(fullfile(root, task, 'results', file), 'result'), 'result');

%% Task 1
r = ld('Task1_Kinova', 'kinovaGen3_kinova_default_getTransform_result.mat');
makeTrajectoryVideo(kinova, r, {'Task 1: Kinova Gen3'}, fullfile(outDir, 'task1_kinova.mp4'));
makeOptimizationVideo(kinova, r, fullfile(outDir, 'task1_optimisation.mp4'), ...
    struct('title', 'Task 1: STOMP iterations (dotted = straight-line initial guess)'));

%% Task 2
r = ld('Task2_NewRobot', sprintf('%s_task2_%s_getTransform_result.mat', rc.robotName, rc.robotName));
makeTrajectoryVideo(panda, r, {'Task 2: Franka Emika Panda'}, fullfile(outDir, 'task2_panda.mp4'));

%% Task 4
for s = 1:3
    r = ld('Task4_Scenarios', sprintf('%s_t4_%d_PoE_result.mat', rc.robotName, s));
    makeTrajectoryVideo(panda, r, {sprintf('Task 4 S%d: %s', s, r.scene.description)}, ...
        fullfile(outDir, sprintf('task4_S%d.mp4', s)));
end
makeOptimizationVideo(panda, r, fullfile(outDir, 'task4_S3_optimisation.mp4'), ...
    struct('title', 'Task 4 S3: STOMP iterations (dotted = straight-line initial guess)'));

%% Task 5: without | with the constraint
rOff = ld('Task5_EEConstraint', sprintf('%s_t5_noCon_PoE_result.mat', rc.robotName));
rOn  = ld('Task5_EEConstraint', sprintf('%s_t5_withCon_PoE_result.mat', rc.robotName));
makeTrajectoryVideo(panda, {rOff, rOn}, {'without constraint', ...
    sprintf('with constraint (w_con = %g)', rOn.cfg.w_con)}, fullfile(outDir, 'task5_compare.mp4'), ...
    struct('showEEAxis', true, 'title', 'Task 5: EE y-axis (orange) kept upright'));

%% Bonus: joint-angle wrap
rN = ld('Bonus_STOMPExtensions', 'kinovaGen3_bonus_wrap_naive_PoE_result.mat');
rS = ld('Bonus_STOMPExtensions', 'kinovaGen3_bonus_wrap_shortest_PoE_result.mat');
makeTrajectoryVideo(kinova, {rN, rS}, {'naive linspace (base turns 5.6 rad)', ...
    'shortest angle (base turns 0.7 rad)'}, fullfile(outDir, 'bonus_wrap_compare.mp4'), ...
    struct('title', 'Base joint 2.8 -> -2.8 rad across \pm\pi'));
