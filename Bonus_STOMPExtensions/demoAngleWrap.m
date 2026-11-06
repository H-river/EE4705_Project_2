function tbl = demoAngleWrap(outDir)
% DEMOANGLEWRAP  Joint-angle discontinuity at +-pi (handout tip): naive vs shortest-angle
% initialisation on the Kinova, whose joints 1/3/5/7 are continuous.
%
% Scene: the Task-1 start/goal configurations with the base joint turned to +2.8 and
% -2.8 rad, i.e. the arm works BEHIND the robot and the base crosses +-pi. A plain
% linspace (original initTrajectory) turns the base the long way round, 5.6 rad through
% the front of the robot; the shortest-angle line turns it 0.68 rad. A second box is put
% on the short path (waypoint 10) so that both initial guesses collide and STOMP has to
% plan in both cases. Same seed for both runs.

if nargin < 1, outDir = fullfile(fileparts(mfilename('fullpath')), 'results'); end
if ~exist(outDir, 'dir'), mkdir(outDir); end

cfg = defaultConfig();
cfg.fkMode = 'PoE';
cfg.outDir = outDir;
cfg.displayAnimation = false;
model = buildRobotModel('kinovaGen3', 'EndEffector_Link', cfg.fkMode);

%% Scene: Task-1 configurations, base joint across +-pi
scene = scene_kinova_default();
[q0, qT] = solveInitFinal(model, scene);
q0(1) = 2.8;   qT(1) = -2.8;
scene.q0 = q0;  scene.qT = qT;
thShort = initTrajectory(q0, qT, cfg.nDiscretize, model.isContinuous);
pMid = tform2trvec(getTransform(model.robot, thShort(:, round(cfg.nDiscretize / 2)), model.eeName));
scene.obstacles(end + 1) = struct('type', 'box', 'center', pMid, 'dims', [0.12 0.12 0.12]);
scene.axisLimits = [-1 1 -1 1 -0.2 1.2];
scene.viewAngle  = [-150 25];

%% Plan with both initialisations
modes = {'naive', 'shortest'};
runs = cell(1, 2);
row = cell(2, 8);
for k = 1:2
    c = cfg;  c.shortestAngleInit = strcmp(modes{k}, 'shortest');
    s = scene;  s.name = ['bonus_wrap_' modes{k}];
    s.description = sprintf('Kinova, base joint 2.8 -> -2.8 rad, %s initialisation', modes{k});
    rng(0);
    r = runPipeline(model, s, c);
    runs{k} = r;
    row(k, :) = {string(modes{k}), sum(abs(diff(r.thetaInit(1, :)))), sum(abs(diff(r.theta(1, :)))), ...
        sum(vecnorm(diff(r.theta, 1, 2))), eePathLength(model, r.theta), r.nIter, ...
        r.initInCollision, r.isTrajectoryInCollision};
    plotConfigOverlay(model, r.theta, r.world, s, struct('title', ...
        sprintf('Angle wrap: %s initialisation', modes{k}), ...
        'saveFile', fullfile(outDir, sprintf('bonus_wrap_%s_overlay.png', modes{k}))));
end
tbl = cell2table(row, 'VariableNames', {'init', 'baseTravelInit_rad', 'baseTravelResult_rad', ...
    'jointPathLength_rad', 'EEpathLength_m', 'iterations', 'initCollides', 'resultCollides'});
disp(tbl);
writetable(tbl, fullfile(outDir, 'bonus_wrap.csv'));

%% Base-joint angle along the trajectory
fig = figure('Name', 'bonus_wrap_joint1', 'Position', [100 100 700 380]);
ax = axes(fig);  hold(ax, 'on');  grid(ax, 'on');
col = [0.85 0.33 0.10; 0.00 0.45 0.74];
for k = 1:2
    plot(ax, runs{k}.thetaInit(1, :), '--', 'Color', col(k, :), 'LineWidth', 1.2, ...
        'DisplayName', [modes{k} ' init']);
    plot(ax, runs{k}.theta(1, :), '-o', 'Color', col(k, :), 'LineWidth', 1.5, 'MarkerSize', 4, ...
        'DisplayName', [modes{k} ' result']);
end
yline(ax, [pi -pi], ':k', {'\pi', '-\pi'}, 'HandleVisibility', 'off');
xlabel(ax, 'Waypoint index');  ylabel(ax, 'Base joint angle [rad]');
title(ax, 'Base joint 2.8 \rightarrow -2.8 rad: naive vs shortest-angle initialisation');
legend(ax, 'Location', 'best');
exportgraphics(fig, fullfile(outDir, 'bonus_wrap_joint1.png'), 'Resolution', 200);
end


function L = eePathLength(model, theta)
% EE path length [m] (PoE / getTransform via the shared FK)
P = zeros(3, size(theta, 2));
for t = 1:size(theta, 2)
    [~, T] = updateJointsWorldPosition(model, theta(:, t));
    P(:, t) = T{end}(1:3, 4);
end
L = sum(vecnorm(diff(P, 1, 2)));
end
