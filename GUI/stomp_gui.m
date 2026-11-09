function out = stomp_gui()
% STOMP_GUI  Interactive front end: choose a scenario and the planner options, plan with
% STOMP, watch the trajectory and read the result (Run: open this file and press F5).
%
% Only a front end: it calls the same functions as the task scripts (buildRobotModel,
% solveInitFinal, buildVoxelWorld, runSTOMP, checkTrajCollision[Dense]) and changes no
% planner code. It saves nothing.
%
% Planning time: a few seconds with PoE. With getTransform a long run (Task 4/5, up to
% 200 iterations) can take a few minutes, and a run cannot be interrupted.
%
% Same seed -> same result in this window. The iteration count can differ slightly from
% the task scripts for the same seed: they also draw the initial figure before planning,
% and that advances MATLAB's random number stream.
%
% Owner: Jiamo

thisDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisDir, '..', 'common')));

rc = robotConfig();
% label, robot, scene, iteration cap, constraint on, w_con
S = { ...
    'Task 1: Kinova, original scene',        'kinova', @scene_kinova_default, 50,  false, 10
    'Task 2: Panda, Task-1-like scene',      'panda',  @scene_newrobot_base,  50,  false, 10
    'Task 4 S1: over the wall',              'panda',  @scene_t4_1,           200, false, 10
    'Task 4 S2: under the shelf',            'panda',  @scene_t4_2,           200, false, 10
    'Task 4 S3: around the column',          'panda',  @scene_t4_3,           200, false, 10
    'Task 5: upright end-effector',          'panda',  @scene_t5,             200, true,  10 };

% view presets: label, azimuth, elevation (NaN = the scenario's own view)
VIEWS = { ...
    'Scenario default',  NaN, NaN
    '3D front-right',    45,  25
    '3D front-left',     135, 25
    'Front (from +x)',   90,  0
    'Side (from -y)',    0,   0
    'Top',               0,   90 };

models = containers.Map();
curLimits = [];       % axis limits of the 3D view
dragging = false;  dragStart = [0 0];  dragView = [0 0];   % mouse rotation state
last = [];            % result of the latest run (for Replay)
animToken = 0;        % increased to stop a running animation

%% Window
fig = figure('Name', 'STOMP planner - EE4705 Group 9', 'NumberTitle', 'off', 'MenuBar', 'none', ...
    'ToolBar', 'none', 'Color', 'w', 'Position', [60 60 1220 720], 'CloseRequestFcn', @onClose);
panel = uipanel(fig, 'Units', 'normalized', 'Position', [0.008 0.015 0.262 0.97], ...
    'BackgroundColor', [0.96 0.97 0.98], 'BorderType', 'line', 'Title', 'Setup', 'FontWeight', 'bold');
y = 0.965;  rowH = 0.043;  gap = 0.008;

ctl.scene = addPopup('Scenario', S(:, 1)', @onScene);
ctl.fk    = addPopup('Forward kinematics', {'PoE', 'getTransform'}, []);
addHeading('Planner');
ctl.limits   = addCheck('Enforce joint limits', true);
ctl.shortest = addCheck('Shortest-angle initialisation', true);
ctl.reuse    = addCheck('Reuse 5 best samples', true);
ctl.patience = addEdit('Convergence patience P', '3');
ctl.maxIter  = addEdit('Maximum iterations', '50');
ctl.seed     = addEdit('Random seed', '0');
addHeading('End-effector constraint (y-axis upright)');
ctl.con  = addCheck('Use constraint', false);
ctl.wcon = addEdit('Weight w_con', '10');

y = y - gap;
btnRun = uicontrol(panel, 'Style', 'pushbutton', 'String', 'Run', 'Units', 'normalized', ...
    'Position', [0.05 y - 0.058 0.56 0.058], 'FontSize', 12, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.20 0.45 0.75], 'ForegroundColor', 'w', 'Callback', @onRun);
btnReplay = uicontrol(panel, 'Style', 'pushbutton', 'String', 'Replay', 'Units', 'normalized', ...
    'Position', [0.65 y - 0.058 0.30 0.058], 'FontSize', 11, 'Enable', 'off', 'Callback', @onReplay);
y = y - 0.058 - gap;

addHeading('Result');
txtResult = uicontrol(panel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.05 0.012 0.90 y - 0.012], 'HorizontalAlignment', 'left', 'FontSize', 10, ...
    'BackgroundColor', [0.96 0.97 0.98], 'String', 'Choose a scenario and press Run.');

%% View controls, above the plot
uicontrol(fig, 'Style', 'text', 'String', 'View', 'Units', 'normalized', ...
    'Position', [0.335 0.952 0.035 0.03], 'FontSize', 10, 'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
ctl.view = uicontrol(fig, 'Style', 'popupmenu', 'String', VIEWS(:, 1)', 'Units', 'normalized', ...
    'Position', [0.37 0.955 0.15 0.035], 'FontSize', 10, 'BackgroundColor', 'w', 'Callback', @(~, ~) applyView());
ctl.zoom = uicontrol(fig, 'Style', 'checkbox', 'String', 'Zoom to workspace', 'Value', 1, 'Units', 'normalized', ...
    'Position', [0.535 0.955 0.14 0.035], 'FontSize', 10, 'BackgroundColor', 'w', 'Callback', @(~, ~) applyView());
uicontrol(fig, 'Style', 'text', 'String', 'Drag the plot to rotate', 'Units', 'normalized', ...
    'Position', [0.69 0.952 0.20 0.03], 'FontSize', 9, 'BackgroundColor', 'w', ...
    'ForegroundColor', [0.4 0.4 0.4], 'HorizontalAlignment', 'left');

ax3 = [];
axQ = axes('Parent', fig, 'Position', [0.335 0.075 0.64 0.17]);
styleCost();
newRobotAxes();
onScene();
set(fig, 'WindowButtonDownFcn', @onMouseDown, 'WindowButtonMotionFcn', @onMouseMove, ...
    'WindowButtonUpFcn', @onMouseUp);

if nargout > 0            % handles for scripted testing
    out = struct('fig', fig, 'controls', ctl, 'run', @() onRun(), 'replay', @() onReplay(), ...
        'selectScene', @selectScene, 'lastResult', @getLast, 'applyView', @applyView, 'axes3D', @getAx3);
end

    function r = getLast()
        r = last;
    end

    function a = getAx3()
        a = ax3;
    end

%% Control builders
    function h = addPopup(label, items, cb)
        addLabel(label, 0.022);
        h = uicontrol(panel, 'Style', 'popupmenu', 'String', items, 'Units', 'normalized', ...
            'Position', [0.05 y - rowH 0.90 rowH], 'FontSize', 10, 'BackgroundColor', 'w');
        if ~isempty(cb), set(h, 'Callback', cb); end
        y = y - rowH - gap;
    end

    function h = addCheck(label, value)
        h = uicontrol(panel, 'Style', 'checkbox', 'String', label, 'Value', value, 'Units', 'normalized', ...
            'Position', [0.05 y - rowH * 0.8 0.90 rowH * 0.8], 'FontSize', 10, 'BackgroundColor', [0.96 0.97 0.98]);
        y = y - rowH * 0.8 - gap * 0.5;
    end

    function h = addEdit(label, value)
        uicontrol(panel, 'Style', 'text', 'String', label, 'Units', 'normalized', ...
            'Position', [0.05 y - rowH * 0.85 0.62 rowH * 0.72], 'FontSize', 10, 'HorizontalAlignment', 'left', ...
            'BackgroundColor', [0.96 0.97 0.98]);
        h = uicontrol(panel, 'Style', 'edit', 'String', value, 'Units', 'normalized', ...
            'Position', [0.70 y - rowH * 0.85 0.25 rowH * 0.85], 'FontSize', 10, 'BackgroundColor', 'w');
        y = y - rowH * 0.85 - gap * 0.6;
    end

    function addHeading(label)
        y = y - gap;
        addLabel(label, 0.03, 'bold');
    end

    function addLabel(label, h, weight)
        if nargin < 3, weight = 'normal'; end
        uicontrol(panel, 'Style', 'text', 'String', label, 'Units', 'normalized', ...
            'Position', [0.05 y - h 0.90 h], 'FontSize', 10, 'FontWeight', weight, ...
            'HorizontalAlignment', 'left', 'BackgroundColor', [0.96 0.97 0.98]);
        y = y - h - gap * 0.3;
    end

%% Callbacks
    function selectScene(idx)
        set(ctl.scene, 'Value', idx);
        onScene();
    end

    function onScene(~, ~)
        k = get(ctl.scene, 'Value');
        set(ctl.maxIter, 'String', num2str(S{k, 4}));
        set(ctl.con, 'Value', S{k, 5});
        set(ctl.wcon, 'String', num2str(S{k, 6}));
    end

    function onRun(~, ~)
        animToken = animToken + 1;                 % stop an animation still playing
        set([btnRun btnReplay], 'Enable', 'off');
        set(txtResult, 'String', 'Planning ...');
        drawnow;
        try
            o = readOptions();
            k = get(ctl.scene, 'Value');
            model = getModel(S{k, 2}, o.fk);
            sc = S{k, 3}();
            cfg = makeCfg(o, S{k, 2}, rc);
            [q0, qT] = solveInitFinal(model, sc);
            [vw, world] = buildVoxelWorld(sc);
            rng(o.seed);
            r = quietPlan(model, q0, qT, vw, cfg);
            r.world = world;  r.scene = sc;  r.label = S{k, 1};  r.model = model;  r.cfg = cfg;
            r.showTilt = cfg.useEEConstraint || k == size(S, 1);   % only meaningful for Task 5
            r.info = evaluate(model, r, world, cfg);
            last = r;
            set(txtResult, 'String', resultText(r));
            plotCost(r);
            playback(r);
        catch err
            set(txtResult, 'String', ['Error: ' err.message]);
        end
        if isvalid(fig)
            set(btnRun, 'Enable', 'on');
            if ~isempty(last), set(btnReplay, 'Enable', 'on'); end
        end
    end

    function onReplay(~, ~)
        if isempty(last), return; end
        animToken = animToken + 1;
        [az, el] = view(ax3);                      % keep the angle the user rotated to
        playback(last, [az el]);
    end

    function onClose(~, ~)
        animToken = animToken + 1;
        delete(fig);
    end

%% Reading the controls
    function o = readOptions()
        fkItems = get(ctl.fk, 'String');
        o.fk = fkItems{get(ctl.fk, 'Value')};
        o.limits   = logical(get(ctl.limits, 'Value'));
        o.shortest = logical(get(ctl.shortest, 'Value'));
        o.reuse    = logical(get(ctl.reuse, 'Value'));
        o.useCon   = logical(get(ctl.con, 'Value'));
        o.patience = readNumber(ctl.patience, 'Convergence patience', 1, 100, true);
        o.maxIter  = readNumber(ctl.maxIter, 'Maximum iterations', 1, 2000, true);
        o.seed     = readNumber(ctl.seed, 'Random seed', 0, 2^32 - 1, true);
        o.wcon     = readNumber(ctl.wcon, 'Weight w_con', 0, 1e6, false);
    end

    function m = getModel(robot, fk)
        key = [robot '_' fk];
        if ~isKey(models, key)
            set(txtResult, 'String', 'Loading the robot model ...');  drawnow;
            if strcmp(robot, 'kinova')
                models(key) = buildRobotModel('kinovaGen3', 'EndEffector_Link', fk);
            else
                models(key) = buildRobotModel(rc.robotName, rc.eeName, fk);
            end
            set(txtResult, 'String', 'Planning ...');  drawnow;
        end
        m = models(key);
    end

%% Drawing
    function newRobotAxes()
        % a fresh axes for every run: the two robots do not share show()'s cached graphics
        % (show() also adds a small xyz-triad axes, so remove every axes except the cost plot)
        old = findall(fig, 'Type', 'axes');
        delete(old(old ~= axQ));
        ax3 = axes('Parent', fig, 'Position', [0.335 0.31 0.64 0.62]);
        grid(ax3, 'on');  xlabel(ax3, 'x [m]');  ylabel(ax3, 'y [m]');  zlabel(ax3, 'z [m]');
    end

%% Mouse rotation of the 3D plot (figure-level callbacks, so it also works while the
% animation is playing, unlike MATLAB's rotate3d mode)
    function onMouseDown(~, ~)
        if isempty(ax3) || ~isvalid(ax3) || isempty(last), return; end
        p = get(fig, 'CurrentPoint');
        box = getpixelposition(ax3, true);
        if p(1) < box(1) || p(1) > box(1) + box(3) || p(2) < box(2) || p(2) > box(2) + box(4), return; end
        [az, el] = view(ax3);
        dragging = true;  dragStart = p;  dragView = [az el];
    end

    function onMouseMove(~, ~)
        if ~dragging || isempty(ax3) || ~isvalid(ax3), return; end
        d = get(fig, 'CurrentPoint') - dragStart;
        view(ax3, dragView(1) - 0.4 * d(1), min(max(dragView(2) - 0.4 * d(2), -89), 89));
    end

    function onMouseUp(~, ~)
        dragging = false;
    end

    function applyView(keep)
        % Set the limits and the camera of the 3D plot from the View controls.
        if isempty(last) || isempty(ax3) || ~isvalid(ax3), return; end
        if get(ctl.zoom, 'Value')
            curLimits = fitLimits(last);
        else
            curLimits = last.scene.axisLimits;
        end
        axis(ax3, curLimits);
        set(ax3, 'DataAspectRatio', [1 1 1]);
        k = get(ctl.view, 'Value');
        if nargin >= 1 && ~isempty(keep)
            view(ax3, keep(1), keep(2));
        elseif isnan(VIEWS{k, 2})
            view(ax3, last.scene.viewAngle(1), last.scene.viewAngle(2));
        else
            view(ax3, VIEWS{k, 2}, VIEWS{k, 3});
        end
    end

    function styleCost()
        grid(axQ, 'on');  xlabel(axQ, 'Iteration');  ylabel(axQ, 'Total cost Q');
    end

    function plotCost(r)
        cla(axQ);
        plot(axQ, 1:numel(r.Q_time), r.Q_time, '-', 'Color', [0.00 0.45 0.74], 'LineWidth', 1.8);
        styleCost();
        xlim(axQ, [1 max(numel(r.Q_time), 2)]);
    end

    function playback(r, keep)
        if nargin < 2, keep = []; end
        token = animToken;
        newRobotAxes();
        hold(ax3, 'on');
        hObs = gobjects(0);
        for i = 1:numel(r.world)
            [~, p] = show(r.world{i}, 'Parent', ax3);
            set(p, 'LineStyle', 'none', 'FaceColor', [0.85 0.20 0.15], 'FaceAlpha', 0.45);
            if i == 1, hObs = p; end
        end
        model = r.model;
        Pinit = eePath(model, r.thetaInit);
        Q = densify(r.theta, 6);
        P = eePath(model, Q);
        hInit = plot3(ax3, Pinit(1, :), Pinit(2, :), Pinit(3, :), ':', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.5);
        hS = plot3(ax3, P(1, 1), P(2, 1), P(3, 1), 'o', 'MarkerSize', 9, 'MarkerFaceColor', [0.20 0.70 0.25], 'MarkerEdgeColor', 'k');
        hG = plot3(ax3, P(1, end), P(2, end), P(3, end), 's', 'MarkerSize', 9, 'MarkerFaceColor', [0.90 0.15 0.15], 'MarkerEdgeColor', 'k');
        trail = plot3(ax3, NaN, NaN, NaN, '-', 'Color', [0.1 0.1 0.1], 'LineWidth', 2);
        legend(ax3, [trail, hInit, hS, hG, hObs], {'planned EE path', 'initial guess (joint-space line)', ...
            'start', 'goal', 'obstacle'}, 'Location', 'northeast', 'FontSize', 8, 'AutoUpdate', 'off');
        applyView(keep);
        title(ax3, r.label, 'Interpreter', 'none');
        arrow = gobjects(1);
        for f = 1:size(Q, 2)
            if ~isvalid(fig) || token ~= animToken, return; end
            show(model.robot, Q(:, f), 'Parent', ax3, 'PreservePlot', false, 'FastUpdate', true, 'Frames', 'off');
            set(trail, 'XData', P(1, 1:f), 'YData', P(2, 1:f), 'ZData', P(3, 1:f));
            if r.cfg.useEEConstraint
                T = getTransform(model.robot, Q(:, f), model.eeName);
                a = 0.12 * T(1:3, 2);  p0 = T(1:3, 4);
                if isgraphics(arrow), delete(arrow); end
                arrow = quiver3(ax3, p0(1), p0(2), p0(3), a(1), a(2), a(3), 0, 'Color', [0.93 0.55 0.05], ...
                    'LineWidth', 3, 'MaxHeadSize', 0.8);
            end
            axis(ax3, curLimits);                  % View controls can change it while playing
            drawnow limitrate;
            pause(0.02);
        end
        drawnow;
    end
end


%% Local functions (no shared state)
function v = readNumber(h, name, lo, hi, isInt)
v = str2double(get(h, 'String'));
if ~isscalar(v) || isnan(v) || v < lo || v > hi || (isInt && v ~= round(v))
    kind = 'a number';
    if isInt, kind = 'a whole number'; end
    error('%s must be %s between %.10g and %.10g.', name, kind, lo, hi);
end
end

function cfg = makeCfg(o, robot, rc)
cfg = defaultConfig();
cfg.fkMode = o.fk;
cfg.displayAnimation = false;
cfg.verbose = false;
cfg.enforceJointLimits = o.limits;
cfg.shortestAngleInit = o.shortest;
if o.reuse, cfg.nReuse = 5; else, cfg.nReuse = 0; end
cfg.convergencePatience = o.patience;
cfg.maxIter = o.maxIter;
cfg.useEEConstraint = o.useCon;
if o.useCon, cfg.w_con = o.wcon; else, cfg.w_con = 0; end
if strcmp(robot, 'panda')                 % Kinova keeps the Task-1 defaults
    cfg.sphereRadius = rc.sphereRadius;
    cfg.safetyMargin = rc.safetyMargin;
end
end

function r = quietPlan(model, q0, qT, vw, cfg) %#ok<INUSD,STOUT> (used inside evalc)
evalc('r = runSTOMP(model, q0, qT, vw, cfg);');     % hide the per-run prints
end

function info = evaluate(model, r, world, cfg)
init = checkTrajCollision(model.robot, r.thetaInit, world);
way  = checkTrajCollision(model.robot, r.theta, world);
btw  = checkTrajCollisionDense(model.robot, r.theta, world, cfg.denseCheckSubsteps);
lim  = model.jointLimits;
info.initColliding = nnz(init);
info.wayColliding  = nnz(way);
info.betweenColliding = nnz(btw);
info.limitViolation = max([0; r.theta(:) - repmat(lim(:, 2), size(r.theta, 2), 1); ...
                           repmat(lim(:, 1), size(r.theta, 2), 1) - r.theta(:)]);
tilt = zeros(1, size(r.theta, 2));
for t = 1:size(r.theta, 2)
    T = getTransform(model.robot, r.theta(:, t), model.eeName);
    tilt(t) = atan2d(norm(cross(T(1:3, 2), [0; 0; 1])), T(3, 2));
end
info.maxTilt = max(tilt);
end

function s = resultText(r)
i = r.info;
if i.wayColliding == 0 && i.betweenColliding == 0
    verdict = 'COLLISION-FREE';
else
    verdict = 'IN COLLISION';
end
s = {verdict, '', ...
    sprintf('Iterations:  %d of max %d', r.nIter, r.cfg.maxIter), ...
    sprintf('Planning time:  %.1f s (%.0f ms/iter)', r.totalTime, 1000 * mean(r.iterTime)), ...
    sprintf('Final cost Q:  %.2f', r.Q_time(end)), ...
    sprintf('Initial guess colliding:  %d / %d waypoints', i.initColliding, size(r.theta, 2)), ...
    sprintf('Result colliding:  %d waypoints, %d segments', i.wayColliding, i.betweenColliding), ...
    sprintf('Worst joint-limit violation:  %.3f rad', i.limitViolation)};
if r.showTilt
    s{end + 1} = sprintf('Max tilt of EE y-axis:  %.1f deg', i.maxTilt);
end
end

function L = fitLimits(r)
% Axis limits around the robot's trajectory and the obstacles, with a margin for the links.
N = size(r.theta, 2);
pts = zeros(0, 3);
for t = 1:N
    X = updateJointsWorldPosition(r.model, r.theta(:, t));
    Y = updateJointsWorldPosition(r.model, r.thetaInit(:, t));
    pts = [pts; X(:, 1:3); Y(end, 1:3)]; %#ok<AGROW>
end
pts = [pts; 0 0 0];
for i = 1:numel(r.scene.obstacles)
    ob = r.scene.obstacles(i);
    switch ob.type
        case 'box'
            half = norm(ob.dims) / 2 * [1 1 1];        % safe for rotated boxes too
            if ~isfield(ob, 'rpy') || isempty(ob.rpy), half = ob.dims(:)' / 2; end
        case 'sphere'
            half = ob.dims(1) * [1 1 1];
        otherwise                                      % cylinder: [radius height]
            half = [ob.dims(1) ob.dims(1) ob.dims(2) / 2];
    end
    pts = [pts; ob.center - half; ob.center + half]; %#ok<AGROW>
end
pad = 0.15;
lo = min(pts, [], 1);  hi = max(pts, [], 1);
L = [lo(1) - pad, hi(1) + pad, lo(2) - pad, hi(2) + pad, lo(3) - 0.05, hi(3) + pad];
end

function Qd = densify(theta, nSub)
N = size(theta, 2);
Qd = zeros(size(theta, 1), (N - 1) * nSub + 1);
a = (0:nSub - 1) / nSub;
for i = 1:N - 1
    Qd(:, (i - 1) * nSub + (1:nSub)) = theta(:, i) + (theta(:, i + 1) - theta(:, i)) .* a;
end
Qd(:, end) = theta(:, end);
end

function P = eePath(model, Q)
P = zeros(3, size(Q, 2));
for t = 1:size(Q, 2)
    T = getTransform(model.robot, Q(:, t), model.eeName);
    P(:, t) = T(1:3, 4);
end
end
