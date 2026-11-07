%% compare_patience.m -- convergence patience P = 1 vs P = 3 on Task 1 and Task 2
% Uses the CURRENT code and defaults (extensions etc.), changes only cfg.convergencePatience.
% Calls runSTOMP directly: no task script, no defaultConfig and no saved result is modified.

clear; clc; close all;
root = fileparts(fileparts(mfilename('fullpath')));        % this file lives in tools/
addpath(genpath(fullfile(root, 'common')));

plainTask1 = false;   % true if the group decides Task 1 = plain STOMP (option a)
seeds      = 0;       % e.g. 0:4 for a small seed study
patience   = [1 3];

rc = robotConfig();
cases = { ...   % label | model | scene | sphere radius | safety margin
    'Task 1 Kinova', buildRobotModel('kinovaGen3', 'EndEffector_Link', 'getTransform'), ...
    scene_kinova_default(), 0.05, 0.05;
    'Task 2 Panda',  buildRobotModel(rc.robotName, rc.eeName, 'getTransform'), ...
    scene_newrobot_base(), rc.sphereRadius, rc.safetyMargin };

rows = table();
for c = 1:size(cases, 1)
    model = cases{c, 2};  scene = cases{c, 3};
    [q0, qT]    = solveInitFinal(model, scene);
    [vw, world] = buildVoxelWorld(scene);
    for P = patience
        cfg = defaultConfig();
        cfg.fkMode = 'getTransform';
        cfg.sphereRadius = cases{c, 4};  cfg.safetyMargin = cases{c, 5};
        cfg.convergencePatience = P;  cfg.verbose = (c == 1 && P == 3);   % print Task 1, P = 3 only
        if c == 1 && plainTask1
            cfg.shortestAngleInit = false;  cfg.enforceJointLimits = false;  cfg.nReuse = 0;
        end
        for s = seeds
            rng(s);
            r = runSTOMP(model, q0, qT, vw, cfg);
            coll = any(checkTrajCollision(model.robot, r.theta, world)) || ...
                any(checkTrajCollisionDense(model.robot, r.theta, world, cfg.denseCheckSubsteps));
            rows = [rows; table(string(cases{c, 1}), P, s, r.nIter, r.totalTime, ...
                r.Q_time(end), r.Q_time(end) - r.RAR_time(end), coll, ...
                100 * meshClearance(model.robot, r.theta, world), ...
                'VariableNames', {'task', 'patience', 'seed', 'nIter', 'time_s', ...
                'finalQ', 'obstacleCost', 'collides', 'minClearance_cm'})]; %#ok<AGROW>
        end
    end
end
disp(rows)

function d = meshClearance(robot, theta, world)
% Smallest mesh distance robot <-> obstacles over all waypoints [m]
d = inf;
for t = 1:size(theta, 2)
    [~, sep] = checkCollision(robot, theta(:, t), world, 'IgnoreSelfCollision', 'on', 'Exhaustive', 'on');
    d = min(d, min(sep(:), [], 'omitnan'));
end
end