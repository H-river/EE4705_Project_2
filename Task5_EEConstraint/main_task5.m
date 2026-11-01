%% Task 5 -- End-effector orientation constraint ("carry a cup of coffee")
% Owner: T5
%
% Cost:  common/stomp/stompConstraintCost.m      (TODO[T5-1])
% Scene: common/scenes/scene_t5.m                (TODO[T5-2a], upright start/goal)
% Shared plots: common/viz/plotConfigOverlay.m, plotCostHistory.m (TODO[T5-3a/b])
% Same scene, same seed, run twice: without and with the constraint.

clear; clc; close all;
thisDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisDir, '..', 'common')));
addpath(thisDir);

rc = robotConfig();
cfg = defaultConfig();
cfg.fkMode       = 'PoE';            % switch to 'getTransform' until T3 is done
cfg.sphereRadius = rc.sphereRadius;
cfg.outDir       = fullfile(thisDir, 'results');
cfg.displayAnimation = false;
cfg.eeAxis    = 2;                   % EE y-axis
cfg.worldAxis = [0; 0; 1];           % kept upright

model = buildRobotModel(rc.robotName, rc.eeName, cfg.fkMode);
scene = scene_t5();

%% Without constraint
cfgOff = cfg;  cfgOff.useEEConstraint = false;
scene.name = 't5_noCon';
rng(0); resOff = runPipeline(model, scene, cfgOff);

%% With constraint
cfgOn = cfg;  cfgOn.useEEConstraint = true;
cfgOn.w_con = 100;   %% TODO[T5-2b] (Owner: T5) tune: too small -> no effect,
                     % too large -> obstacle term ignored / collisions. Sweep a few
                     % values and put the trade-off in the report.
scene.name = 't5_withCon';
rng(0); resOn = runPipeline(model, scene, cfgOn);

%% TODO[T5-2c] (Owner: T5) Before/after comparison (the key Task-5 result)
% - plotEEAxisDeviation({resOff, resOn}, model, cfg, {'no constraint', 'with constraint'})
%   (angle between EE axis and world z at every waypoint);
% - plotConfigOverlay for both runs side by side;
% - note any cost: more iterations, longer path, higher final obstacle cost.
