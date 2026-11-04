%% test_stompConstraintCost.m -- checks for common/stomp/stompConstraintCost.m
% Put this file in Task5_EEConstraint/ and press Run.
%   Parts 1-8 : pure maths, need only the Robotics System Toolbox transform helpers
%               (axang2tform, eul2tform). No STOMP code needed.
%   Part 9    : error handling.
%   Part 10   : the Task-5 scene (needs robotConfig, buildRobotModel, solveInitFinal).
%   Part 11   : consistency with stompTrajCost (needs TODO[T1-2] done by T12).
% Every check prints PASS / FAIL; a summary is printed at the end.

clear; clc; close all;
thisDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisDir, '..', 'common')));
addpath(thisDir);

ok  = logical([]);
cfg = struct();                       % minimal cfg -> defaults: y-axis, world z, 'l1'
modes = {'l1', 'l2sq', 'angle'};

% Rotation about world x by +90 deg maps the EE y-axis [0 1 0] onto world z [0 0 1],
% so this pose is "perfectly upright" for the default constraint.
Tup = axang2tform([1 0 0 pi/2]);

%% 1) Upright pose -> cost 0 in every mode
for m = modes
    c = stompConstraintCost(Tup, setfield(cfg, 'constraintType', m{1})); %#ok<SFLD>
    ok(end+1) = report(sprintf('upright pose gives 0 [%s]', m{1}), abs(c) < 1e-12); %#ok<SAGROW>
end

%% 2) Known tilt: rotate about x by (90 - th) deg -> EE y-axis = [0, sin(th), cos(th)]
th = deg2rad(30);
Ttilt = axang2tform([1 0 0 pi/2 - th]);
expected = struct('l1', sin(th) + 1 - cos(th), 'l2sq', 2*(1 - cos(th)), 'angle', th);
for m = modes
    c = stompConstraintCost(Ttilt, setfield(cfg, 'constraintType', m{1})); %#ok<SFLD>
    ok(end+1) = report(sprintf('30 deg tilt matches closed form [%s]: %.6f', m{1}, c), ...
        abs(c - expected.(m{1})) < 1e-12); %#ok<SAGROW>
end

%% 3) Upside-down (y-axis = -z): the worst case
Tdown = axang2tform([1 0 0 -pi/2]);
ok(end+1) = report('upside-down: l1 = 2',    abs(stompConstraintCost(Tdown, setfield(cfg,'constraintType','l1'))    - 2)  < 1e-12);
ok(end+1) = report('upside-down: l2sq = 4',  abs(stompConstraintCost(Tdown, setfield(cfg,'constraintType','l2sq'))  - 4)  < 1e-12);
ok(end+1) = report('upside-down: angle = pi', abs(stompConstraintCost(Tdown, setfield(cfg,'constraintType','angle')) - pi) < 1e-12);

%% 4) Spinning the EE about the constrained axis must NOT change the cost
% (post-multiplying by a rotation about the body y-axis keeps column 2 of R fixed)
Tspin = Tup * axang2tform([0 1 0 0.7]);
ok(end+1) = report('spin about the constrained axis keeps cost 0', ...
    abs(stompConstraintCost(Tspin, cfg)) < 1e-12);

%% 5) Cost grows monotonically with the tilt (0..90 deg)
tilts = deg2rad(0:5:90);
for m = modes
    cc = arrayfun(@(t) stompConstraintCost(axang2tform([1 0 0 pi/2 - t]), ...
                  setfield(cfg, 'constraintType', m{1})), tilts); %#ok<SFLD>
    ok(end+1) = report(sprintf('cost strictly increases with tilt [%s]', m{1}), all(diff(cc) > 0)); %#ok<SAGROW>
end

%% 6) Other axes / other world directions
cfg1 = cfg;  cfg1.eeAxis = 1;  cfg1.worldAxis = [1; 0; 0];
cfg3 = cfg;  cfg3.eeAxis = 3;  cfg3.worldAxis = [0; 0; 1];
cfgX = cfg;  cfgX.eeAxis = 1;  cfgX.worldAxis = [0; 0; 1];
ok(end+1) = report('eeAxis 1 vs world x on identity pose = 0', abs(stompConstraintCost(eye(4), cfg1)) < 1e-12);
ok(end+1) = report('eeAxis 3 vs world z on identity pose = 0', abs(stompConstraintCost(eye(4), cfg3)) < 1e-12);
ok(end+1) = report('eeAxis 1 vs world z on identity pose: l1 = 2', abs(stompConstraintCost(eye(4), cfgX) - 2) < 1e-12);

%% 7) A non-unit worldAxis is normalised
cfgBig = cfg;  cfgBig.worldAxis = [0; 0; 5];
ok(end+1) = report('worldAxis [0 0 5] behaves like [0 0 1]', ...
    abs(stompConstraintCost(Ttilt, cfgBig) - stompConstraintCost(Ttilt, cfg)) < 1e-12);

%% 8) Random poses vs an independent formula (acos, written differently)
rng(1);
maxErrAng = 0;  maxErrL2 = 0;
for i = 1:200
    T = eul2tform(2*pi*rand(1, 3) - pi);
    a = T(1:3, 2);
    refAng = acos(max(-1, min(1, a(3))));
    refL2  = 2 - 2*a(3);                       % |a - z|^2 for unit a
    maxErrAng = max(maxErrAng, abs(stompConstraintCost(T, setfield(cfg,'constraintType','angle')) - refAng)); %#ok<SFLD>
    maxErrL2  = max(maxErrL2,  abs(stompConstraintCost(T, setfield(cfg,'constraintType','l2sq'))  - refL2));  %#ok<SFLD>
end
ok(end+1) = report(sprintf('angle vs acos on 200 random poses (max err %.1e)', maxErrAng), maxErrAng < 1e-8);
ok(end+1) = report(sprintf('l2sq  vs 2-2*a_z on 200 random poses (max err %.1e)', maxErrL2), maxErrL2 < 1e-12);

%% 9) Bad input must raise an error (not return a silent wrong number)
ok(end+1) = report('error: eeAxis = 4',                 throws(@() stompConstraintCost(Tup, struct('eeAxis', 4))));
ok(end+1) = report('error: unknown constraintType',     throws(@() stompConstraintCost(Tup, struct('constraintType', 'bogus'))));
ok(end+1) = report('error: zero worldAxis',             throws(@() stompConstraintCost(Tup, struct('worldAxis', [0; 0; 0]))));
ok(end+1) = report('error: T_ee is 3x3 not 4x4',        throws(@() stompConstraintCost(eye(3), cfg)));

%% 10) Task-5 scene: start and goal must be upright, straight-line joint path should not be
rc    = robotConfig();
model = buildRobotModel(rc.robotName, rc.eeName, 'getTransform');
scene = scene_t5();
[q0, qT] = solveInitFinal(model, scene);
cfgA = defaultConfig();  cfgA.constraintType = 'angle';
tiltDeg = @(q) rad2deg(stompConstraintCost(getTransform(model.robot, q, model.eeName), cfgA));
fprintf('  start tilt %.3f deg | goal tilt %.3f deg | straight-line midpoint tilt %.3f deg\n', ...
    tiltDeg(q0), tiltDeg(qT), tiltDeg((q0 + qT)/2));
ok(end+1) = report('scene_t5 start is upright (< 1 deg)', tiltDeg(q0) < 1);
ok(end+1) = report('scene_t5 goal is upright (< 1 deg)',  tiltDeg(qT) < 1);
% The midpoint tilt is printed, not asserted: if it is ~0, the constraint has nothing
% to fix in this scene and the before/after comparison will be unconvincing.

%% 11) Consistency with stompTrajCost (needs TODO[T1-2] done, otherwise T{end} is a placeholder)
cfgI = defaultConfig();
cfgI.fkMode = 'getTransform';  cfgI.eeAxis = 2;  cfgI.worldAxis = [0; 0; 1];
N = cfgI.nDiscretize;
[Rm, ~, ~]  = stompSmoothingMatrices(N, cfgI.sigmaScale);
voxelWorld  = buildVoxelWorld(scene);
theta       = initTrajectory(q0, qT, N);
cOff = cfgI;  cOff.useEEConstraint = false;
cOn  = cfgI;  cOn.useEEConstraint  = true;  cOn.w_con = 100;
S0 = stompTrajCost(model, theta, Rm, voxelWorld, cOff);
S1 = stompTrajCost(model, theta, Rm, voxelWorld, cOn);
qc = zeros(1, N);
for i = 1:N
    qc(i) = stompConstraintCost(getTransform(model.robot, theta(:, i), model.eeName), cOn);
end
ok(end+1) = report('Stheta(with) - Stheta(without) = w_con * qc at every waypoint', ...
    max(abs((S1 - S0) - cOn.w_con * qc)) < 1e-9);

%% Summary
fprintf('\n%d / %d checks passed.\n', sum(ok), numel(ok));


function ok = report(name, cond)
ok = all(cond);
if ok, tag = 'PASS'; else, tag = 'FAIL'; end
fprintf('[%s] %s\n', tag, name);
end

function tf = throws(f)
tf = false;
try
    f();
catch
    tf = true;
end
end
