function scene = scene_t5()
% SCENE_T5  Task-5 scene: "carry a cup of coffee" around an obstacle.
%
% Start and goal EE poses are built so that the constrained EE axis (cfg.eeAxis,
% default y) is EXACTLY aligned with the world z axis (= upright) at both ends, as the
% handout suggests ("adjust the initial and final orientation of the end-effector to
% align with your desired orientation"). Between start and goal the planner has to keep
% that axis upright, while a box sits on the straight-line path and forces a detour.
%
% Why the tool points horizontally: if EE-y is vertical, EE-x and EE-z must both lie in
% the horizontal plane (the three axes are orthonormal). Here the "forward" column
% (EE-z for eeAxis = 2) points radially away from the robot base at the start and at
% the goal, which is a natural, easily reachable arm posture.
%
% Depends only on files that already exist (scene_newrobot_base -> scene_kinova_default,
% robotConfig). Nothing from Task 1-4 needs to be finished to call this function.
%
% Owner: T5

scene = scene_newrobot_base();       % envSize, voxelSize, axisLimits, viewAngle
scene.name = 't5_upright';           % main_task5 overrides with t5_noCon / t5_withCon

% solveInitFinal prefers scene.q0 / scene.qT over taskInit / taskFinal. Make sure no
% joint-space start/goal inherited from the base scene overrides our upright poses.
if isfield(scene, 'q0'), scene = rmfield(scene, 'q0'); end
if isfield(scene, 'qT'), scene = rmfield(scene, 'qT'); end

%% Design parameters (tuned for the Kinova placeholder; re-tune if the Task-2 robot differs)
eeAxis  = 2;                 % MUST equal cfg.eeAxis in main_task5.m  (1 = x, 2 = y, 3 = z)
r       = 0.50;              % horizontal distance of start / goal from the base axis [m]
hEE     = 0.30;              % EE height of start / goal [m]
azStart = deg2rad(-35);      % azimuth of the start about the base z axis
azGoal  = deg2rad(60);       % azimuth of the goal

% Box on the arc between start and goal (azimuth ~11 deg, same radius), same size as the
% Task-1 cube. The straight-line joint interpolation sweeps through it, so the planner
% must lift the EE over it (or go around) while keeping the axis upright.
boxCenter = [0.49 0.10 0.30];
boxDims   = [0.16 0.16 0.16];

%% Start / goal poses: EE axis eeAxis = world z, forward column radially outward
pStart = [r * cos(azStart); r * sin(azStart); hEE];
pGoal  = [r * cos(azGoal);  r * sin(azGoal);  hEE];

scene.taskInit  = uprightTform(pStart, azStart, eeAxis);
scene.taskFinal = uprightTform(pGoal,  azGoal,  eeAxis);

%% Obstacle
scene.obstacles = struct('type',   {'box'}, ...
                         'center', {boxCenter}, ...
                         'dims',   {boxDims});

scene.description = ['Upright EE axis at start and goal; box on the sweep path forces ' ...
                     'a detour that tilts the EE unless the constraint is active.'];
end


function T = uprightTform(p, azimuth, eeAxis)
% UPRIGHTTFORM  4x4 pose at position p whose EE axis number eeAxis equals world z.
%   Columns of R are the EE body axes (x, y, z) in the world frame. With a = eeAxis:
%     R(:, a)         = up        = [0; 0; 1]
%     R(:, a+1)       = forward   = [cos(az); sin(az); 0]   (horizontal, indices mod 3)
%     R(:, a+2)       = up x forward                         (keeps R right-handed)
%   For eeAxis = 2: R = [-sin(az) 0 cos(az); cos(az) 0 sin(az); 0 1 0].
up      = [0; 0; 1];
forward = [cos(azimuth); sin(azimuth); 0];

R = zeros(3);
R(:, eeAxis)              = up;
R(:, mod(eeAxis,     3) + 1) = forward;
R(:, mod(eeAxis + 1, 3) + 1) = cross(up, forward);

T = eye(4);
T(1:3, 1:3) = R;
T(1:3, 4)   = p(:);
end
