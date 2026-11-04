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
cfg.fkMode       = 'getTransform';            % switch to 'getTransform' until T3 is done
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

%% With constraint -- weight sweep (TODO[T5-2b])
cfgOn = cfg;  cfgOn.useEEConstraint = true;  cfgOn.verbose = false;
wList  = [0 10 30 100 300 1000];
maxDev = zeros(size(wList));
for i = 1:numel(wList)
    c = cfgOn;  c.w_con = wList(i);
    scn = scene;  scn.name = sprintf('t5_w%g', wList(i));
    rng(0);  r = runPipeline(model, scn, c);  close all;
    ang = zeros(1, size(r.theta, 2));
    for t = 1:size(r.theta, 2)
        [~, T] = updateJointsWorldPosition(model, r.theta(:, t));
        ang(t) = acosd(max(-1, min(1, T{end}(1:3, cfg.eeAxis)' * cfg.worldAxis(:))));
    end
    maxDev(i) = max(ang);
    fprintf('w_con = %5g | max tilt %6.2f deg | collision %d | iters %d\n', ...
        wList(i), maxDev(i), r.isTrajectoryInCollision, r.nIter);
end

% Pick a value from the printed lines, then set it here and run the final case:
cfgOn.w_con = 100;  cfgOn.verbose = true;     % <- replace 100 with your choice
scene.name = 't5_withCon';
rng(0); resOn = runPipeline(model, scene, cfgOn);

%% TODO[T5-2c] (Owner: T5) Before/after comparison (the key Task-5 result)
% - plotEEAxisDeviation({resOff, resOn}, model, cfg, {'no constraint', 'with constraint'})
%   (angle between EE axis and world z at every waypoint);
% - plotConfigOverlay for both runs side by side;
% - note any cost: more iterations, longer path, higher final obstacle cost.
