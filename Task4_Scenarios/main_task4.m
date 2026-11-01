%% Task 4 -- Custom collision-avoidance scenarios
% Owner: Jiamo (T34)
%
% Scenarios: common/scenes/scene_t4_1.m .. scene_t4_3.m  (TODO[T4-2])
% Multi-obstacle voxelisation: common/world/buildVoxelWorld.m (TODO[T4-1a..d])
% Uses the Task-2 robot + PoE FK (Tasks 3/4 build on each other).

clear; clc; close all;
thisDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisDir, '..', 'common')));

rc = robotConfig();
cfg = defaultConfig();
cfg.fkMode       = 'PoE';            % switch to 'getTransform' until T3 is done
cfg.sphereRadius = rc.sphereRadius;
cfg.outDir       = fullfile(thisDir, 'results');
cfg.displayAnimation = false;

model = buildRobotModel(rc.robotName, rc.eeName, cfg.fkMode);

sceneFns = {@scene_t4_1, @scene_t4_2, @scene_t4_3};
results  = cell(1, numel(sceneFns));
for s = 1:numel(sceneFns)
    scene = sceneFns{s}();
    results{s} = runPipeline(model, scene, cfg);

    % Handout: "Do not initialize trivial paths, i.e., collision-free initialization."
    if ~results{s}.initInCollision
        warning('%s: straight-line initialisation is collision-free -> scenario too easy.', scene.name);
    end
end

%% TODO[T4-3] (Owner: Jiamo) Results table + figures for report / slides
% Per scenario: #obstacles, init collides?, result collides?, #iterations, total time,
% final cost, min clearance along the path (optional: min sEDT over spheres).
% Robustness (optional, strong for grading): run each scenario with 5-10 rng seeds and
% report success rate.
% Figures: plotConfigOverlay per scenario, plotCostHistory(results, {'S1','S2','S3'}).
% Videos: set cfg.recordResultVideo = true for the slides.
