%% Task 2 -- STOMP on a different manipulator, scenario similar to Task 1
% Owner: T12
%
% The robot is chosen in common/config/robotConfig.m (TODO[T2-1]) and the scene in
% common/scenes/scene_newrobot_base.m (TODO[T2-4]). Tasks 3-5 pick both up
% automatically, so finish T2-1 early and tell the group.

clear; clc; close all;
thisDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisDir, '..', 'common')));

rc = robotConfig();
if ~rc.isDecided
    warning('Task 2 robot not chosen yet (robotConfig.isDecided = false) -- running Kinova.');
end

cfg = defaultConfig();
cfg.fkMode       = 'getTransform';
cfg.sphereRadius = rc.sphereRadius;
cfg.outDir       = fullfile(thisDir, 'results');

model = buildRobotModel(rc.robotName, rc.eeName, cfg.fkMode);
scene = scene_newrobot_base();

%% TODO[T2-5] (Owner: T12) Sanity checks for the new robot before planning:
% - disp(model.jointBodyNames) / model.jointTypes : fixed bodies excluded? order right?
% - figure; show(model.robot); hold on; plot the collision spheres from
%   stompRobotSphere(updateJointsWorldPosition(model, q0), cfg.sphereRadius)
%   to check they cover every link (screenshot -> report).

result = runPipeline(model, scene, cfg);

%% TODO[T2-6] (Owner: T12) Report figures (overlay + cost) via plotConfigOverlay / plotCostHistory.
