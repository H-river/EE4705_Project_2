function model = buildRobotModel(robotName, eeName, fkMode)
% BUILDROBOTMODEL  Load a robot once and pre-compute everything the planner needs.
%
% model fields (shared interface -- do not rename without telling the group):
%   .name            robot name or URDF path
%   .robot           rigidBodyTree, DataFormat 'column' (show / ik / checkCollision / getTransform)
%   .eeName          end-effector body name
%   .baseName        base frame name (all FK results are w.r.t. this frame)
%   .jointBodyNames  1 x nJoints cell, bodies whose joint is NOT fixed, in config order
%   .fkBodyNames     1 x nFK cell = [jointBodyNames, eeName]; row k of X / cell k of T
%   .nJoints, .nFK
%   .jointTypes      1 x nJoints cell, 'revolute' | 'prismatic' | 'continuous'
%   .fkMode          'getTransform' | 'PoE'
%   .S               6 x nJoints space screw axes      (PoE only, from computeScrewAxes)
%   .Mhome           4 x 4 x nFK home poses of fkBodies (PoE only, from computeScrewAxes)
%
% Notes vs. the original sample code:
%   * The original used BodyNames{1..nJoints}. That silently picks FIXED bodies on
%     robots whose chain starts with fixed joints (e.g. UR), and on Kinova it drops
%     EndEffector_Link (so the last link had no collision spheres and Task 5 had no
%     EE frame). Here we keep only movable bodies and always append the EE.

if nargin < 3 || isempty(fkMode)
    fkMode = 'getTransform';
end

model.name = robotName;

%% Load
if endsWith(robotName, '.urdf', 'IgnoreCase', true)
    %% TODO[T2-1b] (Owner: T12) External URDF: add the mesh path argument
    % (importrobot(..., 'MeshPath', ...)) so show() renders the meshes.
    model.robot = importrobot(robotName, 'DataFormat', 'column');
else
    model.robot = loadrobot(robotName, 'DataFormat', 'column');
end

model.eeName   = char(eeName);
model.baseName = model.robot.BaseName;

%% Movable bodies (Bodies{k} is slow, but this runs only once)
nB = model.robot.NumBodies;
isMovable  = false(1, nB);
jointTypes = cell(1, nB);
for k = 1:nB
    jointTypes{k} = model.robot.Bodies{k}.Joint.Type;
    isMovable(k)  = ~strcmp(jointTypes{k}, 'fixed');
end
model.jointBodyNames = model.robot.BodyNames(isMovable);
model.jointTypes     = jointTypes(isMovable);
model.nJoints        = numel(model.jointBodyNames);

nCfg = numel(homeConfiguration(model.robot));
assert(model.nJoints == nCfg, ...
    'buildRobotModel: %d movable bodies but configuration has %d entries.', model.nJoints, nCfg);

if ~any(strcmp(model.robot.BodyNames, model.eeName))
    error('buildRobotModel: end-effector body "%s" not found in %s.', model.eeName, robotName);
end
model.fkBodyNames = [model.jointBodyNames, {model.eeName}];
model.nFK         = numel(model.fkBodyNames);

%% Kinematics mode
model.fkMode = fkMode;
model.S      = [];
model.Mhome  = [];
if strcmp(fkMode, 'PoE')
    % Screw axes depend only on the home (zero) configuration -> compute ONCE.
    [model.S, model.Mhome] = computeScrewAxes(model);
end
end
